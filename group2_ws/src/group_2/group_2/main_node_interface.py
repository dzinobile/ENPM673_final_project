from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import random
from sensor_msgs.msg import Image
# -- Optical Flow
from sensor_msgs.msg import CompressedImage
# -- End Optical Flow
from cv_bridge import CvBridge
import cv2
import torch
import traceback
from geometry_msgs.msg import Twist
import numpy as np 

class MainNode(Node):

    def __init__(self, node_name='main_node'):
    
        super().__init__(node_name)

        self.cv_bridge = CvBridge()

        # --- Optical Flow 
        self.subscription_image = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.main_cb,
            10
        )
        
        self.object_detected = False
        self.frame1 = None 
        self.frame2 = None
        self.smoothed_u = None 
        self.smoothed_v = None 
        self.alpha = 0.8

        self.publisher_flow_image = self.create_publisher(
            Image,
            '/flow_image',
            10
        )

        self.publisher_residual_image = self.create_publisher(
            Image, 
            '/residual_flow_image', 
            10
        )
        
        self.publisher_mask_image = self.create_publisher(
            Image, 
            '/flow_mask_image', 
            10
        )
        
        self.publisher_cmd_vel= self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        self.publisher_processed = self.create_publisher(
            Image, 
            '/final_display', 
            10
        )

    # --- Optical flow Helper Functions ---
    def of_crop(self, gray, wpl=0.15, wph=0.85, hpl=.5, hph=1):
        H, W = gray.shape
        return gray[int(hpl*H):int(hph*H), int(wpl*W):int(wph*W)]
    
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
            self.get_logger().info("Obstacle detected with obstacle flow")
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
        u_est = np.median(u).astype(np.uint8)
        v_est = np.median(v).astype(np.uint8)
        return u_est, v_est
    
    # --- End of Optical Flow Helper Functions

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0   
        msg.angular.z = 0.0  
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info("Stopping the TurtleBot")
        return None

    def horizon_cb(self,msg):
        self.horizon_not_initialized = False
        self.horizon_y = msg.data
        self.get_logger().info(f"Received Horizon value : {self.horizon_y }")
        return None
    
    def main_cb(self,msg):
        try:



            #-----------------------------Optical Flow logic start---------------------------------
            # Optical Flow code
            if isinstance(msg, Image):
                cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            else: # CompressedImage
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                cv_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

            H, W = cv_frame.shape[:2]
            wpl, wph = 0.15, 0.85
            hpl, hph = 0.5, 1.0

            x1 = int(wpl * W)
            x2 = int(wph * W)
            y1 = int(hpl * H)
            y2 = int(hph * H)

            


            if self.frame1 is None: 
                self.frame1 = self.of_standardize(cv_frame)
                return None
            self.frame2 = self.of_standardize(cv_frame)
            flow = self.of_optical_flow()
            if self.smoothed_u is None:
                self.smoothed_u = flow[..., 0]
                self.smoothed_v = flow[..., 1]
            else:
                self.smoothed_u = self.alpha * self.smoothed_u + (1 - self.alpha) * flow[..., 0]
                self.smoothed_v = self.alpha * self.smoothed_v + (1 - self.alpha) * flow[..., 1]

            flow_image_bgr = self.of_flow_image()
            u_est, v_est = self.of_estimate_ego_motion()

            residual = np.copy(flow)
            residual[..., 0] -= u_est 
            residual[..., 1] -= v_est
            self.get_logger().info(f"DEBUG ego: u_est={u_est:.2f}, v_est={v_est:.2f}")
            
            flow_image_bgr, mask_image_gray, self.object_detected = self.of_residual_image(residual)

            cv2.rectangle(
                cv_frame,
                (x1, y1),
                (x2, y2),
                color=(255, 0, 0),
                thickness=2
            )
            # Publish residual flow and mask
            flow_msg = self.cv_bridge.cv2_to_imgmsg(flow_image_bgr, encoding='bgr8')

            self.publisher_flow_image.publish(flow_msg)
            residual_flow_msg = self.cv_bridge.cv2_to_imgmsg(flow_image_bgr, encoding='bgr8')

            self.publisher_residual_image.publish(residual_flow_msg)
            mask_msg = self.cv_bridge.cv2_to_imgmsg(mask_image_gray, encoding='mono8')

            self.publisher_mask_image.publish(mask_msg)


            self.frame1 = self.frame2

            if self.object_detected:
                label = "Unsafe"
                color = (0, 0, 255)
            else:
                label = "Safe"
                color = (0, 255, 0)
            
            cv2.putText(
                cv_frame, 
                label, 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                2, 
                cv2.LINE_AA
            )

            if self.object_detected:
                self.get_logger().info("Object detected")
                self.stop_robot()  # To stop the robot
                # Final display being published
                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame

            #-----------------------------Optical Flow logic end-------------------------------------



            #-----------------------------Regular path following logic start---------------------------



            #-----------------------------Regular path following logic end---------------------------

            # Final display being published
            image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
            self.publisher_processed.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None


