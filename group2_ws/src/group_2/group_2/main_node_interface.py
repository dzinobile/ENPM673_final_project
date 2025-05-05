from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import traceback
from geometry_msgs.msg import Twist

class MainNode(Node):
    def __init__(self, node_name='main_node'):
    
        super().__init__(node_name)
        self.cv_bridge = CvBridge()
        self.subscription_raw_image = self.create_subscription(Image,'/camera/image_raw', self.main_cb,10)
        self.subscription_raw_image = self.create_subscription(Image,'/camera/image_raw', self.main_cb,10)
        self.publisher_cmd_vel= self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_horizon_y = self.create_subscription(Int32,'/horizon_y', self.horizon_cb,10)
        self.publisher_processed = self.create_publisher(Image, '/final_display', 10)
        self.model = torch.hub.load('yolov5','yolov5s', source='local')
        self.model.eval()
        self.horizon_not_initialized = True
        self.horizon_y = None
        self.object_detected = False

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
        if self.horizon_not_initialized or self.horizon_y is None:
            self.get_logger().info("Waiting for horizon finder to find horizon")
            return None
        try:
            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_frame.shape[:2]
            # Drawing the horizon line
            cv2.line(cv_frame, (0, self.horizon_y), (width-1, self.horizon_y), (0,255,0), 2)



            #--------------------------------Stop sign logic start--------------------------------
            #  Run inference
            results = self.model(cv_frame)
            labels = results.names
            detections = results.pred[0]
            stop_sign_detected = False
            for *box, conf, cls in detections:
                label = labels[int(cls)]
                if label.lower() == "stop sign":
                    stop_sign_detected = True
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(cv_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(cv_frame, f"{label} ({conf:.2f})", (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)      
            if stop_sign_detected:
                self.get_logger().info("Stop sign detected")
                self.stop_robot()  # To stop the robot
                # Final display being published
                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame
            #--------------------------------Stop sign logic end--------------------------------



            #-----------------------------Optical Flow logic start---------------------------------
            # Optical Flow code
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


