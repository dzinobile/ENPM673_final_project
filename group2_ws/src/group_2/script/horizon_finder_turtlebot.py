#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int32
import numpy as np
import os
import yaml
from collections import Counter
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import sys
package_share = get_package_share_directory('group_2')  
config_path = os.path.join(package_share, 'config', 'params.yaml')


# Declaration of debug directly folders
DEBUG_DIRS = {
    'original':      '00_original',
    'markers':       '01_markers',
    'corners':       '02_corners',    
    'intersections': '03_intersections',
    'horizon':       '04_horizon',
    'good_horizon' : '05_good_horizon'
}
# Global variables
MAIN_DEBUG_DIR = "ros2_horizon_debug"
###################### Aruco stuff ##################################
# ------------------------------------------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
# ------------------------------------------------------------------



class HorizonfinderNode(Node):
    def __init__(self, node_name='horizon_finder'):
        super().__init__(node_name)

        topic_prefix = '/tb4_1'

        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(CompressedImage,topic_prefix+'/oakd/rgb/preview/image_raw/compressed', self.frame_cb,10)
        self.publish_horizon = self.create_publisher(Float64MultiArray, topic_prefix+'/horizon_line', 10)
        self.publish_good_horizon = self.create_publisher(Float64MultiArray, topic_prefix+'/good_horizon_line', 10)
        self.one_vp_not_published = True
        self.frame_count = 0
        self.horizon_initialized = False
        


        # parameter declaration
        self.declare_parameter('debug', False)

        
        # accessing the parameters
        self.debug = (self.get_parameter('debug').value)


        if self.debug:
            # Creating debug output directories
            for dir_name in DEBUG_DIRS.values():
                os.makedirs(os.path.join(MAIN_DEBUG_DIR, dir_name), exist_ok=True)
        self.get_logger().info("Horizon Initializer initiated")
    
    
    # Method to remove near horizontal and near vertical lines.
    @staticmethod
    def line_from_points(point_1, point_2):
        a = point_2[1] - point_1[1]   
        b = point_1[0] - point_2[0]   
        c = -(a * point_1[0] + b * point_1[1]) #ax + by + c = 0
        return a, b, c
      
    @staticmethod
    def intersection_finder(line_1, line_2):
        # Solving the linear systems
        # ax1 + by1 + c1 = 0
        # ax2 + by2 + c2 = 0
        A = np.array([[line_1[0], line_1[1]], [line_2[0], line_2[1]]])  
        B = np.array([-line_1[2], -line_2[2]])
        try:
            intersection = np.linalg.solve(A, B)
            return intersection
        except np.linalg.LinAlgError:
            return None  # Paralell lines

    @staticmethod
    def horizon_line_drawer(image, vanishing_points):
        h, w = image.shape[:2]
        for (x_value, y_value) in vanishing_points:
            x_point = int(round(x_value))
            y_point = int(round(y_value))
            cv2.circle(image, (x_point, y_point), 5, (0, 0, 255), -1)
            
        if len(vanishing_points) ==  1:
            y_0 = int(round(vanishing_points[0][1]))
            y_0 = np.clip(y_0, 0, h - 1)
            cv2.line(image,(0,   y_0),(w-1, y_0),color=(255, 255, 0),thickness=2)
            return image
        
        # Converting  to numpy array
        vanishing_points_array = np.array(vanishing_points)

        # Fitting  y = mx + c using least squares
        x = vanishing_points_array[:, 0]
        y = vanishing_points_array[:, 1]
        A = np.vstack([x, np.ones_like(x)]).T
        m, c = np.linalg.lstsq(A, y, rcond=None)[0]
        h, w = image.shape[:2]

        # Computing y at left and right edges of the image
        y1 = int(round(m * 0 + c))
        y2 = int(round(m * (w - 1) + c))

        # Clipping y values to image edges
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h - 1, y2))
        cv2.line(image, (0, y1), (w - 1, y2), (255, 255, 0), 2)

            
        return image

    # Callback function of the '/camera/image_raw' topic
    def frame_cb(self, msg):
        # Checking if initialization already done
        if self.horizon_initialized : 
            self.get_logger().info("Already initialized the horizon.")
            self.get_logger().info("Node will now shut down.")
            self.destroy_subscription(self.subscription) 
            self.destroy_node()
            rclpy.shutdown()
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
            aruco_corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # Debug check
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['original'],f"frame_{self.frame_count:04d}.jpg"),cv_frame)

            # Save only if marker is detected
            if aruco_corners:
                marker_frame = cv_frame.copy()
                cv2.aruco.drawDetectedMarkers(marker_frame, aruco_corners, ids)
                if self.debug:
                    cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['markers'],f"frame_{self.frame_count:04d}.jpg"),marker_frame)
                    
                vanishing_points = []

                for corners, marker_id in zip(aruco_corners, ids):
                    corners = corners.reshape(4, 2).astype(int)

                    if self.debug:
                        corner_frame = cv_frame.copy()
                        for i, pt in enumerate(corners):
                            cv2.circle(corner_frame, tuple(pt), 5, (0, 0, 255), -1)
                            cv2.putText(corner_frame, f"{i}", tuple(pt + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                            cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['corners'],f"frame_{self.frame_count:04d}.jpg"), corner_frame)
                            

                    # # Use corner indices 0–1 and 2–3 as edges
                    top_point_1, top_point_2 = corners[0], corners[1]
                    bottom_point_1, bottom_point_2 = corners[2], corners[3]
                    # top_point_1, top_point_2 = corners[0], corners[3]
                    # bottom_point_1, bottom_point_2 = corners[1], corners[2]


                    Line_1 = self.line_from_points(top_point_1, top_point_2)
                    Line_2 = self.line_from_points(bottom_point_1, bottom_point_2)

        
                    intersection = self.intersection_finder(Line_1, Line_2)
                    if intersection is not None:
                        vanishing_points.append(intersection)
                        
                        if self.debug:
                            intersections_frame = cv_frame.copy()
                            cv2.circle(intersections_frame, tuple(intersection.astype(int)), 6, (0, 255, 0), -1)
                            cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['intersections'],f"frame_{self.frame_count:04d}.jpg"), intersections_frame)
                            
                if len(vanishing_points) >= 2:
                    if self.debug:
                        horizon_frame = intersections_frame.copy()
                        horizon_frame =self.horizon_line_drawer(horizon_frame, vanishing_points)
                        cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['horizon'],f"frame_{self.frame_count:04d}.jpg"), horizon_frame)

                    # Checking for good horizon using slope value
                    vanishing_points_array = np.array(vanishing_points)
                    x = vanishing_points_array[:, 0]
                    y = vanishing_points_array[:, 1]
                    A = np.vstack([x, np.ones_like(x)]).T
                    m, c = np.linalg.lstsq(A, y, rcond=None)[0]

                    if abs(m) < 0.05: 
                        self.get_logger().info(f"Frame {self.frame_count}: Good horizon detected.")  
                        flat = [float(v) for pt in vanishing_points for v in pt]
                        msg  = Float64MultiArray(data=flat)
                        self.publish_horizon.publish(msg)
                        self.publish_good_horizon.publish(msg)
                        self.get_logger().info("Published good horizon line details")
                        if self.debug:
                            cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['good_horizon'], f"frame_{self.frame_count:04d}.jpg"), horizon_frame)
                        flat = [float(v) for pt in vanishing_points for v in pt]
                        msg  = Float64MultiArray(data=flat)
                        self.publish_horizon.publish(msg)
                        self.publish_good_horizon.publish(msg)
                        self.get_logger().info("Published good horizon line details")
                        self.get_logger().info("Horizon Initialization complete")
                        self.get_logger().info("Node will now shut down.")
                        self.horizon_initialized = True
                        self.destroy_subscription(self.subscription) 
                        self.destroy_node()
                        rclpy.shutdown()
                        
                if self.one_vp_not_published and len(vanishing_points) ==1 :
                    flat = [float(v) for pt in vanishing_points for v in pt]
                    msg  = Float64MultiArray(data=flat)
                    self.publish_horizon.publish(msg)
                    self.one_vp_not_published = False
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Error noticed: {str(e)}')

   
def main(args=None):
    rclpy.init(args=args)
    node = HorizonfinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()