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
from std_msgs.msg import Bool
import time 

class YoloNode(Node):
    def __init__(self, node_name='yolo_node'):
    
        super().__init__(node_name)


        self.cv_bridge = CvBridge()
        self.subscription_image = self.create_subscription(Image,'/camera/image_raw', self.main_cb,1)
        self.stop_sign_publisher = self.create_publisher(Bool,'/stop_sign',1)
        self.stop_detected = False
        self.model = torch.hub.load('yolov5', 'yolov5n',source='local')  # for yolo v5 nano
        # self.model = torch.hub.load('yolov5', 'yolov5s',source='local')  # for yolo v5s 
        self.model.eval()
      

    def main_cb(self, msg):
        try:

            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_frame)
            labels = results.names
            detections = results.pred[0]
            self.stop_detected = False
            for *box, conf, cls in detections:
                label = labels[int(cls)]
                if label.lower() == "stop sign":
                    self.stop_detected = True     
            if self.stop_detected:
                stop_msg = Bool()
                stop_msg.data = True
                # self.stop_sign_publisher.publish(stop_msg)
                self.get_logger().info("Stop sign detected")
               
            else:
                # self.get_logger().info("No stop sign")
                stop_msg = Bool()
                stop_msg.data = False
                self.stop_sign_publisher.publish(stop_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None