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
import time 
from rclpy.qos import QoSProfile,HistoryPolicy,ReliabilityPolicy,DurabilityPolicy
qos_buffer1 = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
qos_buffer10 = qos_buffer1 = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)

class YoloNode(Node):
    def __init__(self, node_name='yolo_node'):
    
        super().__init__(node_name)


        topic_prefix = '/tb4_1'


        self.cv_bridge = CvBridge()
        self.subscription_image = self.create_subscription(CompressedImage,topic_prefix+'/oakd/rgb/preview/image_raw/compressed', self.main_cb,qos_profile=qos_buffer1)
        self.stop_sign_publisher = self.create_publisher(Bool,topic_prefix+'/stop_sign',1)
        self.stop_detected = False
        self.model = torch.hub.load('yolov5', 'yolov5n',source='local')  # for yolo v5 nano
        self.model.eval()
      

    def main_cb(self, msg):
        try:

            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
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
                self.stop_sign_publisher.publish(stop_msg)
                self.get_logger().info("Stop sign detected")
               
            else:
                stop_msg = Bool()
                stop_msg.data = False
                self.stop_sign_publisher.publish(stop_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None