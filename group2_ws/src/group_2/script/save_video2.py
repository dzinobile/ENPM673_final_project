#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class VideoSaver(Node):
    def __init__(self):
        super().__init__('video_saver')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/tb4_2/oakd/rgb/preview/image_raw/compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.video_writer = cv2.VideoWriter(
            'output_video.mp4',
            cv2.VideoWriter_fourcc(*'mp4v'),  
            24,  # Frame rate 
            (250, 250)  # Resolution 
        )

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.get_logger().info("pringing image")
            self.video_writer.write(cv_image)
           
            
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def destroy_node(self):
        self.video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
