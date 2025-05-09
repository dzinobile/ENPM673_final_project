#!/usr/bin/env python3
import rclpy

# Import the custom PublisherNode class
from group_2.optical_node_interface_gazebo import (
    OpticalNode,
)


def main(args=None):

    rclpy.init(args=args)  
    node = OpticalNode("optical_node")  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown() 


if __name__ == "__main__":
    main() 