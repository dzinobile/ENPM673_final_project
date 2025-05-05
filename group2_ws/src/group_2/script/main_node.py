#!/usr/bin/env python3
import rclpy

# Import the custom PublisherNode class
from group_2.main_node_interface import (
    MainNode,
)



def main(args=None):

    rclpy.init(args=args)  
    node = MainNode("main_node")  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown() 


if __name__ == "__main__":
    main() 