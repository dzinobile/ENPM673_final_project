#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from enpm673_module.enpm673_final_proj import *
class VideoSaver(Node):
    def __init__(self):
        super().__init__('video_saver')
        self.subscription = self.create_subscription(
            Image,
            '/tb4_1/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.video_writer = cv2.VideoWriter(
            'output_video.mp4',
            cv2.VideoWriter_fourcc(*'mp4v'),  
            30,  # Frame rate 
            (250, 250)  # Resolution 
        )

    def image_callback(self, msg):
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
           
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
