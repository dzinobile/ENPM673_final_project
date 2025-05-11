#!/usr/bin/env python3
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import traceback
from geometry_msgs.msg import Twist
import sys
import numpy as np
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool


class OpticalNode(Node):
    def __init__(self, node_name='optical_node'):
    
        super().__init__(node_name)

        self.multi_thread_group = ReentrantCallbackGroup()
        self.cv_bridge = CvBridge()
        self.subscription_image = self.create_subscription(Image,'/camera/image_raw', self.main_cb,10,callback_group=self.multi_thread_group)
        # self.subscription_horizon_line = self.create_subscription(Float64MultiArray, '/horizon_line', self.horizon_cb,1,callback_group=self.multi_thread_group)
        self.stop_publisher = self.create_publisher(Bool,'/stop_robot',1)        # self.horizon_initialized = False
        self.avg_horizon_value = None
        self.vanishing_points = None
        self.object_detected = False
        self.frame1 = None 
        self.frame2 = None
        self.smoothed_u = None 
        self.smoothed_v = None 
        
        self.S = None
        self.HPL = 0.5
        self.HPH  = 1.0
        self.WPL = 0.0
        self.WPH = 1.0
        self.STANDARD_SIZE = (250, 250)
        
        self.publisher_flow_image = self.create_publisher(Image,'/flow_image',10)

        self.publisher_residual_image = self.create_publisher(Image, '/residual_flow_image', 10)
        
        self.publisher_mask_image = self.create_publisher(Image, '/flow_mask_image',10)


    # def horizon_cb(self,msg):
    #     if self.horizon_initialized:
    #         self.destroy_subscription(self.subscription_horizon_line) 
    #         return None
    #     data = msg.data
    #     length    = len(data)//2
    #     vp = [ (data[2*i], data[2*i+1]) for i in range(length)]
    #     self.vanishing_points  = [ (data[2*i], data[2*i+1]) for i in range(length)]
    #     y_values = [y for (_, y) in self.vanishing_points]
    #     self.avg_horizon_value = sum(y_values) / len(y_values)
    #     self.get_logger().info(f"Received Horizon value")
    #     return None

    def convert_gray(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    def crop_roi(self, gray):
        H, W = gray.shape

        # Raw indices
        y1_raw = int(self.HPL * H)
        y2_raw = int(self.HPH * H)
        x1_raw = int(self.WPL * W)
        x2_raw = int(self.WPH * W)

        # Ensure indices are always non-empty
        y1 = min(max(0, y1_raw), H - 1)
        y2 = max(y1 + 1, min(y2_raw, H))
        x1 = min(max(0, x1_raw), W - 1)
        x2 = max(x1 + 1, min(x2_raw, W))

        return gray[y1:y2, x1:x2]

    def standardize(self, frame):
        resized = cv2.resize(frame, self.STANDARD_SIZE, interpolation=cv2.INTER_AREA)
        gray = self.convert_gray(resized)
        return self.crop_roi(gray)

    def optical_flow(self, f1, f2):
        return cv2.calcOpticalFlowFarneback(
            f1, f2, None,
            0.5, 3, 30, 3, 5, 1.2, 0
        )

    def flow_image(self, flow):
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        hsv = np.zeros((flow.shape[0], flow.shape[1], 3), dtype=np.uint8)
        hsv[...,0] = (ang * 180/np.pi/2).astype(np.uint8)
        hsv[...,1] = 255
        hsv[...,2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def estimate_affine_ransac(self, flow, shape, thresh=1.0):
        H, W = shape
        Y, X = np.indices((H,W))
        # Define a points set containing all the (i, j) for the pre-image
        pts1 = np.stack((X.ravel(), Y.ravel()), axis=1).astype(np.float32)
        # Apply the flow translation (u, v) to each (i, j) in the pre-image
        pts2 = np.stack((X.ravel() + flow[...,0].ravel(),
                         Y.ravel() + flow[...,1].ravel()), axis=1).astype(np.float32)
        # Fit an affine transformation that models the transformation of pre-image to image
        M, inliers = cv2.estimateAffinePartial2D(
            pts1, pts2, method=cv2.RANSAC, ransacReprojThreshold=thresh
        )
        if M is None or inliers is None:
            return None, None
        return M, inliers.reshape(H, W)

    def generate_affine_flow(self, shape, M):
        H, W = shape
        Y, X = np.indices((H,W))
        pts = np.stack((X.ravel(), Y.ravel()), axis=1).astype(np.float32)
        warped = cv2.transform(pts.reshape(-1,1,2), M).reshape(H,W,2)
        # Returns the modeled flow
        return warped - np.stack((X,Y), axis=2)
        
    def smooth_mask_ema(self, raw_mask):
        ALPHA = 0.2
        TAU = 0.7
        raw_mask_float = raw_mask.astype(np.float32)
        if self.S is None or self.S.shape != raw_mask_float.shape:
            self.S = raw_mask_float.copy()
        else:
            self.S[:] = ALPHA * raw_mask_float + (1 - ALPHA) * self.S
        return (self.S >= TAU).astype(np.uint8)
        
    def residual_analysis(self, flow, est_flow, inlier_mask, min_percent: float=5.0):
        # Compute the difference in model-fitted flow and computed flow
        residual = flow - est_flow
        # Convert to polar form
        mag, ang  = cv2.cartToPolar(residual[...,0], residual[...,1])
        # Generate an outlier mask from the RANSAC model inliers
        mask = (inlier_mask == 0)
        # Compute the portion full (after some temporal smoothing)
        percent = 100 * mask.mean()
        mask_smoothed = self.smooth_mask_ema(mask)
        percent_smoothed = 100 * mask_smoothed.mean()
        self.get_logger().info(f"Percent smoothed {percent_smoothed: .2f}")
        # If the percentage is large enough, then predict there is a dynamic obstacle
        detected = percent_smoothed > min_percent

        V = (mag/np.max(mag)*255).astype(np.uint8) if np.max(mag)>0 else np.zeros_like(mag, dtype=np.uint8)
        hsv = np.zeros((flow.shape[0], flow.shape[1],3), dtype=np.uint8)
        hsv[...,0] = (ang * 180/np.pi/2).astype(np.uint8)
        hsv[...,1] = 255
        hsv[...,2] = V
        hsv[~mask] = 0 # Inliers are zeroed out in the HSV residual image

        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), (mask*255).astype(np.uint8), detected

    def main_cb(self, msg: Image):

        try:
            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Crop to ROI and convert to grayscale
            if self.frame1 is None:
                self.frame1 = self.standardize(cv_frame)
                return
            frame2 = self.standardize(cv_frame)

            # Handle empty frames
            if frame2.size == 0:
                self.get_logger().error(f"Empty ROI: shape {frame2.shape}")
                self.frame1 = frame2.copy()
                return

            # Compute the optical flow between the two frames
            flow = self.optical_flow(self.frame1, frame2)

            # Model the ego-motion with a RANSAC-fitted affine
            M, inliers = self.estimate_affine_ransac(flow, self.frame1.shape)
            if M is None or inliers is None:
                self.frame1 = frame2.copy()
                return
            
            # Transform the pre-image with the model affine
            est_flow = self.generate_affine_flow(self.frame1.shape, M)

            # 
            residual_img, mask_gray, detected = self.residual_analysis(flow, est_flow, inliers)

            # 
            # self.publisher_flow_image.publish(
            #     self.cv_bridge.cv2_to_imgmsg(self.flow_image(flow), encoding='bgr8')
            # )
            # self.publisher_residual_image.publish(
            #     self.cv_bridge.cv2_to_imgmsg(residual_img, encoding='bgr8')
            # )
            self.publisher_mask_image.publish(
                self.cv_bridge.cv2_to_imgmsg(mask_gray, encoding='mono8')
            )

            # Stop the robot if there's a detection
            stop_msg = Bool()
            stop_msg.data = bool(detected)
            self.stop_publisher.publish(stop_msg)

            # Complete a frame update
            self.frame1 = frame2.copy()

        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {e}")
            self.get_logger().error(traceback.format_exc())

