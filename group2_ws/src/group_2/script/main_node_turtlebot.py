#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import the custom PublisherNode class
from group_2.main_node_interface_turtlebot import (
    MainNode,
)


def main(args=None):

    rclpy.init(args=args)  
    node = MainNode("main_node")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 