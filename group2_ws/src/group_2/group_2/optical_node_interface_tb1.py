#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import traceback
import time
import sys
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile,HistoryPolicy,ReliabilityPolicy
qos_buffer1 = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE)
qos_buffer10 = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,reliability=ReliabilityPolicy.RELIABLE)
class OpticalNode(Node):
    def __init__(self, node_name='optical_node'):
        super().__init__(node_name)

        topic_prefix = '/tb4_1'

        # CV Bridge & threading
        self.cv_bridge = CvBridge()
        self.multi_thread_group = ReentrantCallbackGroup()
        
        # Horizon-line integration
        self.subscription_image = self.create_subscription(CompressedImage,topic_prefix+'/oakd/rgb/preview/image_raw/compressed', self.main_cb,qos_profile=qos_buffer10)
        self.subscription_horizon_line = self.create_subscription(Float64MultiArray, topic_prefix+'/horizon_line', self.horizon_cb,callback_group=self.multi_thread_group,qos_profile=qos_buffer1)
        self.stop_publisher = self.create_publisher(Bool,topic_prefix+'/stop_robot',1)
        self.horizon_initialized = False
        self.avg_horizon_value = None
        self.vanishing_points = None
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
        

        

        # Image subscription
        self.subscription_image = self.create_subscription(CompressedImage,topic_prefix+'/oakd/rgb/preview/image_raw/compressed',self.main_cb,1,callback_group=self.multi_thread_group)

        # self.publisher_flow_image = self.create_publisher(CompressedImage,topic_prefix+'/flow_image',10)

        # self.publisher_residual_image = self.create_publisher(CompressedImage, topic_prefix+'/residual_flow_image', 10)
        
        self.publisher_mask_image = self.create_publisher(CompressedImage, topic_prefix+'/flow_mask_image', 10)

        # Internal state
        self.frame1 = None
        self.alpha  = 0.8

    def horizon_cb(self,msg):
        self.get_logger().info(f"Received Horizon value")
        return None

    def convert_gray(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    def crop_roi(self, gray):
        H, W = gray.shape

        # Raw indices
        y1_raw = int(self.HPL * H)
        y2_raw = int(self.HPH * H)
        x1_raw = int(self.WPL * W)
        x2_raw = int(self.WPH * W)

        # Clamp to ensure non-empty
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
        """
        Fits a 2x3 affine transformation to the openCV-computed optical flow point correspondences, returning the matrix and inlier markings.
        """
        H, W = shape
        Y, X = np.indices((H,W))
        pts1 = np.stack((X.ravel(), Y.ravel()), axis=1).astype(np.float32)
        pts2 = np.stack((X.ravel()+flow[...,0].ravel(),
                         Y.ravel()+flow[...,1].ravel()), axis=1).astype(np.float32)
        M, inliers = cv2.estimateAffinePartial2D(
            pts1, pts2, method=cv2.RANSAC, ransacReprojThreshold=thresh
        )
        if M is None or inliers is None:
            return None, None
        return M, inliers.reshape(H, W)

    def generate_affine_flow(self, shape, M):
        """
        Computes the model-predicted flow image using the affine transform, returning it.
        """
        H, W = shape
        Y, X = np.indices((H,W))
        pts = np.stack((X.ravel(), Y.ravel()), axis=1).astype(np.float32)
        warped = cv2.transform(pts.reshape(-1,1,2), M).reshape(H,W,2)
        return warped - np.stack((X,Y), axis=2)
    
    def smooth_mask_ema(self, raw_mask):
        """
        Applies an exponential moving average filter to the flow mask to smooth it temporally. 
        """
        ALPHA = 0.2
        TAU = 0.7
        raw_mask_float = raw_mask.astype(np.float32)
        if self.S is None or self.S.shape != raw_mask_float.shape:
            self.S = raw_mask_float.copy()
        else:
            self.S[:] = ALPHA * raw_mask_float + (1 - ALPHA) * self.S
        return (self.S >= TAU).astype(np.uint8)

    def residual_analysis(self, flow, est_flow, inlier_mask, min_percent: float=5.0):
        """
        Decides whether or not an obstacle is detected based on the difference (residual) between the affine model flow and the apparent (Farnb.) computed flow at each pixel. The decision is ultimately a matter of counting outliers. 
        """
        residual = flow - est_flow
        mag, ang  = cv2.cartToPolar(residual[...,0], residual[...,1])
        mask = (inlier_mask == 0)
        percent = 100 * mask.mean()
        mask_smoothed = self.smooth_mask_ema(mask)
        percent_smoothed = 100 * mask_smoothed.mean()
        self.get_logger().info(f"Percent smoothed {percent_smoothed: .2f}")
        detected = percent_smoothed > min_percent

        V = (mag/np.max(mag)*255).astype(np.uint8) if np.max(mag)>0 else np.zeros_like(mag, dtype=np.uint8)
        hsv = np.zeros((flow.shape[0], flow.shape[1],3), dtype=np.uint8)
        hsv[...,0] = (ang * 180/np.pi/2).astype(np.uint8)
        hsv[...,1] = 255
        hsv[...,2] = V
        hsv[~mask] = 0
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), (mask*255).astype(np.uint8), detected

    def main_cb(self, msg: Image):
        # if not self.horizon_initialized:
        #     return None

        try:
            #cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Initialize frame 1 and convert to gray and crop
            if self.frame1 is None:
                self.frame1 = self.standardize(cv_frame)
                return

            # Crop frame 2 and convert to gray
            frame2 = self.standardize(cv_frame)

            # Handle the case of an empty ROI
            if frame2.size == 0:
                self.get_logger().error(f"Empty ROI: shape {frame2.shape}")
                self.frame1 = frame2.copy()
                return

            # Compute dense optical flow
            flow = self.optical_flow(self.frame1, frame2)

            # Model the typical flow with an affine transformation
            M, inliers = self.estimate_affine_ransac(flow, self.frame1.shape)
            if M is None or inliers is None:
                self.frame1 = frame2.copy()
                return
            est_flow = self.generate_affine_flow(self.frame1.shape, M)

            # Decide on obstacle based on reprojection error
            residual_img, mask_gray, detected = self.residual_analysis(flow, est_flow, inliers)


            mask_msg = CompressedImage()
            mask_msg.header.stamp = self.get_clock().now().to_msg()
            mask_msg.format = 'jpeg'
            _,buffer = cv2.imencode('.jpg',mask_gray)
            mask_msg.data = np.array(buffer).tobytes()
            self.publisher_mask_image.publish(mask_msg)

            # Publish the robot stop flag
            stop_msg = Bool()
            stop_msg.data = bool(detected)
            self.stop_publisher.publish(stop_msg)

            # Complete a frame update
            self.frame1 = frame2.copy()

        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {e}")
            self.get_logger().error(traceback.format_exc())
