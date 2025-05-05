#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int32
import numpy as np
import os
import yaml
from collections import Counter
from ament_index_python.packages import get_package_share_directory
package_share = get_package_share_directory('group_2')  
config_path = os.path.join(package_share, 'config', 'params.yaml')


# Declaration of debug directly folders
DEBUG_DIRS = {
    'edges': '01_edges',
    'all_lines': '02_all_lines',
    'filtered_lines': '03_filtered_lines',
    'intersection_points': '04_intersection_points',
    'inliers': '05_inliers',
    'final': '06_final'
}
# Global variables
MAIN_DEBUG_DIR = "ros2_horizon_debug"
ANGLE_THRESHOLD = np.pi/90
RANSAC_INLIER_THRESHOLD  = 20
RANSAC_ITERATIONS = 400


class HorizonfinderNode(Node):
    def __init__(self, node_name='horizon_finder'):
        super().__init__(node_name)
        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/tb4_1/oakd/rgb/image_raw', self.frame_cb,10)
        self.horizon_y_publisher = self.create_publisher(Int32, '/tb4_1/horizon_y', 10)
        self.frame_buffer = []
        self.max_frames = 2
        self.frame_count = 0
        self.prev_horizon_x = None
        self.prev_horizon_y = None

        # paramter declaration
        self.declare_parameter('horizon_y', 125)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_angle_resolution', np.pi/180)
        self.declare_parameter('debug', False)

        
        # accessing the parameters
        self.horizon_y = int(self.get_parameter('horizon_y').value)
        self.hough_threshold = int(self.get_parameter('hough_threshold').value)
        self.hough_angle_resolution = float(self.get_parameter('hough_angle_resolution').value)
        self.debug = (self.get_parameter('debug').value)


        if self.debug:
            # Creating debug output directories
            for dir_name in DEBUG_DIRS.values():
                os.makedirs(os.path.join(MAIN_DEBUG_DIR, dir_name), exist_ok=True)
        self.get_logger().info("Horizon Initializer initiated. Waiting for 100 frames to fill up the buffer...")
    
    @staticmethod
    # Method to write the latest paramater values (especially the horizon value to yaml file.)
    def write_param_to_yaml(filename, horizon_y, threshold=50, angle_resolution=0.0174533, debug =True):
            params = {
                '/**': {
                    'ros__parameters': {
                        'horizon_y': horizon_y,
                        'threshold': threshold,
                        'angle_resolution': angle_resolution,
                        'debug': debug
                    }
                }
            }
            with open(filename, 'w') as f:
                yaml.dump(params, f)
    
    # Method to remove near horizontal and near vertical lines.
    @staticmethod
    def horizontal_vertical_remover(line, angle_threshold=ANGLE_THRESHOLD):
        r, theta = line[0]
        theta = theta % np.pi
        return (abs(theta) < angle_threshold) or  (abs(theta - np.pi/2) < angle_threshold) or (abs(theta - np.pi) < angle_threshold)

    @staticmethod
    def intesection_finder(line_1, line_2):
        r_1, theta_1 = line_1[0]
        r_2, theta_2 = line_2[0]
        denominator = np.cos(theta_1) * np.sin(theta_2) - np.cos(theta_2) * np.sin(theta_1)
        # If denominator is close to zero, skip.
        if abs(denominator) < 1e-8:
            return None
        x = (r_1 * np.sin(theta_2) - r_2 * np.sin(theta_1)) / denominator
        y = (r_2 * np.cos(theta_1) - r_1 * np.cos(theta_2)) / denominator
        return (x, y)

    @staticmethod
    # Reference from opencv doc to draw lines after Hough transform
    def draw_lines(frame, lines, color=(0,0,255), length=1000):
        output = frame.copy()
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x_0 = a * rho
                y_0 = b * rho
                pt_1 = (int(x_0 + length*(-b)), int(y_0 + length*(a)))
                pt_2 = (int(x_0 - length*(-b)), int(y_0 - length*(a)))
                cv2.line(output, pt_1, pt_2, color, 1)
        return output


    # Callback function of the '/camera/image_raw' topic
    def frame_cb(self, msg):
        # Checking if initialization already done
        if len(self.frame_buffer) >= self.max_frames:
            self.get_logger().info("Already initialized and went through 100 frames, ignoring further frames.")
            self.get_logger().info("Node will now shut down.")
            self.destroy_subscription(self.subscription) 
            self.destroy_node()
            rclpy.shutdown()
            return

        try:
            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_frame.shape[:2]
            horizon_x, horizon_y  = width // 2, height // 2  # picking the image centre as starting default

            # Edge Detection using Canny (Gray scale --> Gaussian Blur --> Canny detector)
            gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Debug check
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['edges'], f"frame_{self.frame_count:04d}.jpg"), edges)

            # All Detected Lines using Hough
            lines = cv2.HoughLines(edges, 1, self.hough_angle_resolution , self.hough_threshold)
            all_lines_frame = self.draw_lines(cv_frame, lines, (0,0,255))
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['all_lines'], f"frame_{self.frame_count:04d}.jpg"), all_lines_frame)

            # Filtered Lines (non-horizontal/vertical lines)
            filtered_lines = []
            if lines is not None:
                for line in lines:
                    if not self.horizontal_vertical_remover(line):
                        filtered_lines.append(line)

            filtered_lines_frame = self.draw_lines(cv_frame, filtered_lines, (255,0,0))
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['filtered_lines'], f"frame_{self.frame_count:04d}.jpg"), filtered_lines_frame)


            # Prospective Vanishing Points (intersections of all the filtered lines)
            vanishing_point_frame = cv_frame.copy()
            prospective_vanishing_points = []
            for i in range(len(filtered_lines)):
                for j in range(i+1, len(filtered_lines)):
                    intersection_point = self.intesection_finder(filtered_lines[i], filtered_lines[j])
                    if intersection_point is not None:
                        prospective_vanishing_points.append(intersection_point)
                        cv2.circle(vanishing_point_frame, (int(intersection_point[0]), int(intersection_point[1])), 2, (0,0,255), -1)
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['intersection_points'], f"frame_{self.frame_count:04d}.jpg"), vanishing_point_frame)

            # RANSAC 
            RANSAC_frame = cv_frame.copy()
            if prospective_vanishing_points:
                vanishing_points_array = np.array(prospective_vanishing_points)
                vanishing_point_y_vals = vanishing_points_array[:,1]
                best_y, best_inliers = None, 0
                for _ in range(RANSAC_ITERATIONS):
                    y_random = np.random.choice(vanishing_point_y_vals)
                    # Finding inlier count
                    inliers = np.sum(np.abs(vanishing_point_y_vals - y_random) < RANSAC_INLIER_THRESHOLD )
                    if inliers > best_inliers:
                        best_inliers, best_y = inliers, y_random
                if best_y is not None:
                    # Finding actual inliers for maximum inlier count
                    inlier_actual = np.abs(vanishing_point_y_vals - best_y) < RANSAC_INLIER_THRESHOLD
                    inlier_vanishing_points = vanishing_points_array[inlier_actual]
                    for vanishing_point in inlier_vanishing_points:
                        cv2.circle(RANSAC_frame, (int(vanishing_point[0]), int(vanishing_point[1])), 2, (0,255,0), -1)
                    if self.debug:
                        cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['inliers'], f"frame_{self.frame_count:04d}.jpg"), RANSAC_frame)
                    if len(inlier_vanishing_points) > 0:
                        horizon_x = int(np.median(inlier_vanishing_points[:,0]))
                        horizon_y = int(np.median(inlier_vanishing_points[:,1]))

            # Final stage with weighted history averaging to smoothen sudden variations
            if self.prev_horizon_x is not None:
                horizon_x = int(0.8*self.prev_horizon_x + 0.2*horizon_x)
                horizon_y = int(0.8*self.prev_horizon_y + 0.2*horizon_y)
            final_frame = cv_frame.copy()
            cv2.line(final_frame, (0, horizon_y), (width-1, horizon_y), (0,255,0), 2)
            cv2.circle(final_frame, (horizon_x, horizon_y), 5, (0,255,255), -1)
            cv2.putText(final_frame, f"X: {horizon_x}, Y: {horizon_y}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['final'], f"frame_{self.frame_count:04d}.jpg"), final_frame)
            self.prev_horizon_x, self.prev_horizon_y = horizon_x, horizon_y
            self.horizon_y = self.prev_horizon_y 

            # Buffer for counting just 100 frames
            self.frame_count += 1
            if self.horizon_y is not None:
                self.frame_buffer.append(self.horizon_y)

            if len(self.frame_buffer) == self.max_frames:
                avg_y = int(Counter(self.frame_buffer).most_common(1)[0][0])
                try:
                    self.write_param_to_yaml(config_path, avg_y)
                    self.get_logger().info("Parameter file rewritten")
                    msg = Int32()
                    msg.data = int(avg_y)
                    self.horizon_y_publisher.publish(msg)
                    self.get_logger().info(f"Published horizon_y: {avg_y}")
                    self.get_logger().info(f"Initialization complete.Horizon Y: {avg_y}")
                    self.get_logger().info("Node will now shut down.")
                    self.destroy_subscription(self.subscription) 
                    self.destroy_node()
                    rclpy.shutdown()
                except:
                    self.get_logger().error("Unable to write to paramter file")
                    msg = Int32()
                    msg.data = int(avg_y)
                    self.horizon_y_publisher.publish(msg)
                    self.get_logger().info(f"horizon line : {avg_y}")
                    self.get_logger().info(f"Initialization complete.Horizon Y: {avg_y}")
                    self.get_logger().info("Node will now shut down.")
                    self.destroy_subscription(self.subscription) 
                    self.destroy_node()
                    rclpy.shutdown()

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
