#!/usr/bin/env python3
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import random
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import torch
import traceback
from geometry_msgs.msg import TwistStamped
import sys
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

class OpticalNode(Node):
    def __init__(self, node_name='optical_node'):
    
        super().__init__(node_name)

        which_robot = int(sys.argv[1])
        if which_robot == 1:
            topic_prefix = '/tb4_1'
        elif which_robot == 2:
            topic_prefix = '/tb4_2'

        self.cv_bridge = CvBridge()
        self.subscription_image = self.create_subscription(CompressedImage,topic_prefix+'/oakd/rgb/preview/image_raw/compressed', self.main_cb,10)
        self.subscription_horizon_line = self.create_subscription(Float64MultiArray, topic_prefix+'/horizon_line', self.horizon_cb,1)
        self.stop_publisher = self.create_publisher(Bool,topic_prefix+'/stop_robot',1)
        self.horizon_not_initialized = True
        self.x_0 = None 
        self.y_0 = None
        self.x_w = None
        self.y_w = None
        self.object_detected = False
        self.frame1 = None 
        self.frame2 = None
        self.smoothed_u = None 
        self.smoothed_v = None 
        self.alpha = 0.8
        
        self.publisher_flow_image = self.create_publisher(
            CompressedImage,
            topic_prefix+'/flow_image',
            10
        )

        self.publisher_residual_image = self.create_publisher(
            CompressedImage, 
            topic_prefix+'/residual_flow_image', 
            10
        )
        
        self.publisher_mask_image = self.create_publisher(
            CompressedImage, 
            topic_prefix+'/flow_mask_image', 
            10
        )



    def horizon_cb(self,msg):
        self.horizon_not_initialized = False
        self.x_0, self.y_0, self.x_w, self.y_w = msg.data
        self.get_logger().info(f"Received Horizon value")
        return None
    

    def of_crop(self, gray, wpl=0.15, wph=0.85, hpl=.5, hph=1):
        H, W = gray.shape
        return gray[int((self.y_0+self.y_w)/2):int(hph*H), int(wpl*W):int(wph*W)]
    
    def of_convert_gray(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    def of_standardize(self, frame):
        return self.of_crop(self.of_convert_gray(frame))
    
    def of_polar(self, flow):
        return cv2.cartToPolar(flow[..., 0], flow[..., 1])

    def of_flow_image(self):
        mag, ang = cv2.cartToPolar(self.smoothed_u, self.smoothed_v)
        hsv = np.zeros((self.smoothed_u.shape[0], self.smoothed_u.shape[1], 3), dtype=np.uint8)
        hsv[..., 0] = ang * 180/np.pi/2 # 8-bit hue: [0, 360] deg -> [0, 180]
        hsv[..., 1] = 255
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        flow_image_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return flow_image_bgr
    
    def of_residual_image(self, flow):

        detected = False
        # 1. Convert to polar
        mag, ang = self.of_polar(flow)
        # 2. Subtract away the typical value 
        b = np.percentile(mag, 50) # Typical non-ego flow mag of frame
        #b = np.average(mag)
        mag_corr = mag - b 
        mag_corr[mag_corr < 0] = 0 
        
        thresh = 1.0 #Tuning parameter
        mask = (mag_corr > thresh)
        activated_vals = mag_corr[mask]
        if activated_vals.size:
            activated_average = np.average(activated_vals)
        else:
            activated_average = 0.0 
        
        percent_full = 100 * mask.mean()
        self.get_logger().info(f"Percent full: {percent_full: .2f}; Activated Average {activated_average: .2f}")

        if percent_full > 10:
            #self.get_logger().info("Obstacle detected with obstacle flow")
            detected = True 

        mask_image_gray = (mask * 255).astype(np.uint8)

        # Visualization
        V = (mag_corr / np.max(mag_corr) * 255).astype(np.uint8)
        hsv = np.zeros((flow.shape[0], flow.shape[1], 3), dtype=np.uint8)
        hsv[..., 0] = mask * (ang * 180/np.pi/2).astype(np.uint8)
        hsv[..., 1] = mask * 255
        hsv[..., 2] = mask * V
        flow_image_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        #cv2.imshow("Residual image", flow_image_bgr)
        #cv2.imshow("Mask", (mask * 255).astype(np.uint8))
        
        return flow_image_bgr, mask_image_gray, detected
    
    def of_optical_flow(self):
        flow = cv2.calcOpticalFlowFarneback(
                self.frame1, self.frame2, None, .5, 3, 30, 3, 5, 1.2, 0
            ) 
        return flow
    
    def of_estimate_ego_motion(self):
        u, v = self.smoothed_u.flatten(), self.smoothed_v.flatten()
        u_est = float(np.median(u))
        v_est = float(np.median(v))
        return u_est, v_est



    def main_cb(self, msg):


        if self.horizon_not_initialized:
            self.get_logger().info("Waiting for horizon finder to find horizon")
            return None
        try:

            #cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            height, width = cv_frame.shape[:2]

            if self.frame1 is None: 
                self.frame1 = self.of_standardize(cv_frame)
                return None
            self.frame2 = self.of_standardize(cv_frame)


            # Drawing the horizon line
            cv2.line(cv_frame, (0,int(self.y_0)), (width,int(self.y_w)), (0,0,255), 2)

            #-----------------------------Optical Flow logic start---------------------------------
            # Optical Flow code

            wpl, wph = 0.15, 0.85
            hpl, hph = 0.5, 1.0

            x1 = int(wpl * width)
            x2 = int(wph * width)
            y1 = int((self.y_0+self.y_w)/2)
            y2 = int(hph * height)


            flow = self.of_optical_flow()
            if self.smoothed_u is None:
                self.smoothed_u = flow[..., 0]
                self.smoothed_v = flow[..., 1]
            else:
                self.smoothed_u = self.alpha * self.smoothed_u + (1 - self.alpha) * flow[..., 0]
                self.smoothed_v = self.alpha * self.smoothed_v + (1 - self.alpha) * flow[..., 1]

            flow_image_bgr = self.of_flow_image()


            
            #flow_msg = self.cv_bridge.cv2_to_imgmsg(flow_image_bgr, encoding='bgr8')
            flow_msg = CompressedImage()
            flow_msg.header.stamp = self.get_clock().now().to_msg()
            flow_msg.format = 'jpeg'
            _,buffer = cv2.imencode('.jpg',flow_image_bgr)
            flow_msg.data = np.array(buffer).tobytes()
            self.publisher_flow_image.publish(flow_msg)

            u_est, v_est = self.of_estimate_ego_motion()

            residual = np.copy(flow)
            residual[..., 0] -= u_est 
            residual[..., 1] -= v_est
            self.get_logger().info(f"DEBUG ego: u_est={u_est:.2f}, v_est={v_est:.2f}")
            
            res_image_bgr, mask_image_gray, self.object_detected = self.of_residual_image(residual)

            #residual_flow_msg = self.cv_bridge.cv2_to_imgmsg(res_image_bgr, encoding='bgr8')
            residual_flow_msg = CompressedImage()
            residual_flow_msg.header.stamp = self.get_clock().now().to_msg()
            residual_flow_msg.format = 'jpeg'
            _,buffer = cv2.imencode('.jpg',res_image_bgr)
            residual_flow_msg.data = np.array(buffer).tobytes()
            self.publisher_residual_image.publish(residual_flow_msg)

            mask_msg = CompressedImage()
            mask_msg.header.stamp = self.get_clock().now().to_msg()
            mask_msg.format = 'jpeg'
            _,buffer = cv2.imencode('.jpg',mask_image_gray)
            mask_msg.data = np.array(buffer).tobytes()
            self.publisher_mask_image.publish(mask_msg)


            self.frame1 = self.frame2



            if self.object_detected:
                stop_msg = Bool()
                self.get_logger().info("Object detected with optical flow")
                stop_msg.data = True
                self.stop_publisher.publish(stop_msg)
            else:
                stop_msg = Bool()
                stop_msg.data = False
                self.stop_publisher.publish(stop_msg)
                
            #-----------------------------Optical Flow logic end-------------------------------------



        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None


