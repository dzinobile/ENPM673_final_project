#!/usr/bin/env python3
import rclpy

# Import the custom PublisherNode class
from group_2.yolo_node_interface_turtlebot import (
    YoloNode,
)


def main(args=None):

    rclpy.init(args=args)  
    node = YoloNode("yolo_node")  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown() 


if __name__ == "__main__":
    main() 