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
from cv2 import aruco
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
package_share = get_package_share_directory('group_2')  
config_path = os.path.join(package_share, 'config', 'params.yaml')


# Declaration of debug directly folders
DEBUG_DIRS       = {
    'original'    : '01_original',
    'detect_charuco': '02_detect_charuco',
    'corners'     : '03_corners',
    'lines'       : '04_lines',
    'horizon'     : '05_horizon'
}
# Global variables
MAIN_DEBUG_DIR = "ros2_horizon_debug"
BOARD_SIZE       = (5, 7)     
SQUARE_LENGTH    = 0.029
MARKER_LENGTH    = 0.019
DICTIONARY       = aruco.DICT_4X4_50
RANSAC_ITERARATIONS   = 1000
RANSAC_INLIER_THRESHOLD = 30  # in pixels

class HorizonfinderNode(Node):
    def __init__(self, node_name='horizon_finder'):
        super().__init__(node_name)
        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/tb4_2/oakd/rgb/image_raw/compressed', self.frame_cb,10)
        self.publish_horizon = self.create_publisher(Float64MultiArray, '/tb4_2/horizon_line', 1)
        self.horizon_initialized = False
        self.frame_count=0

        self.declare_parameter('debug', False)
        self.debug = (self.get_parameter('debug').value)

        # Initialize Charuco Board
        self.charuco_dict  = aruco.getPredefinedDictionary(DICTIONARY)
        self.charuco_board = aruco.CharucoBoard(
            (BOARD_SIZE[0], BOARD_SIZE[1]),
            SQUARE_LENGTH, MARKER_LENGTH,
            self.charuco_dict
        )


        if self.debug:
            # Creating debug output directories
            for dir_name in DEBUG_DIRS.values():
                os.makedirs(os.path.join(MAIN_DEBUG_DIR, dir_name), exist_ok=True)

        self.get_logger().info("Horizon finder Initializer initiated")
    
    
    # @staticmethod
    def detect_charuco(self,cv_frame, board):
        gray_image = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray_image, self.charuco_dict)
        charuco_frame = cv_frame.copy()

        if ids is not None and len(ids) > 0:
            retval, corners, ids = aruco.interpolateCornersCharuco(corners, ids, gray_image, board)
            if retval > 0:
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,100, 1e-4)
                corners = cv2.cornerSubPix(gray_image, corners, (5,5), (-1,-1), criteria)
                charuco_frame = aruco.drawDetectedCornersCharuco(charuco_frame, corners, ids)
                return True, corners, ids, charuco_frame
        return False, None, None, charuco_frame
    
    @staticmethod
    def get_charuco_lines_fixed5X7(self,corners, ids):
        points      = corners.reshape(-1,2)
        ids_flat = ids.flatten().astype(int)

        # set-1: vertical (4 columns)
        column_id_sets = [
            [ 0, 4, 8,12,16,20],
            [ 1, 5, 9,13,17,21],
            [ 2, 6,10,14,18,22],
            [ 3, 7,11,15,19,23]
        ]
        # set-2: horizontal (6 rows)
        horiz_id_sets = [
            [ 0, 1, 2, 3],
            [ 4, 5, 6, 7],
            [ 8, 9,10,11],
            [12,13,14,15],
            [16,17,18,19],
            [20,21,22,23]
        ]

        row_lines = []
        for row_idx, id_list in enumerate(horiz_id_sets):
            mask = np.isin(ids_flat, id_list)
            row_pts = points[mask]
            if len(row_pts) < 2:
                print(f"skip horizontal row {row_idx}: only {len(row_pts)} pts")
                self.get_logger().info(f"skip horizontal row {row_idx}: only {len(row_pts)} pts")
                continue

            vx, vy, x0, y0 = cv2.fitLine(row_pts.astype(np.float32),cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            row_lines.append((vx,vy,x0,y0))

        column_lines = []
        for col_idx, id_list in enumerate(column_id_sets):
            mask = np.isin(ids_flat, id_list)
            col_pts = points[mask]
            if len(col_pts) < 2:
                self.get_logger().info(f"skip vertical col {col_idx}: only {len(col_pts)} pts")
                continue
            vx, vy, x0, y0 = cv2.fitLine(col_pts.astype(np.float32),cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            column_lines.append((vx,vy,x0,y0))

        return row_lines, column_lines

    @staticmethod
    def set_intersections(lines):
        points = []
        n = len(lines)
        for i in range(n):
            vx1,vy1,x1,y1 = lines[i]
            for j in range(i+1, n):
                vx2,vy2,x2,y2 = lines[j]
                det = vx1*vy2 - vy1*vx2
                if abs(det) < 1e-6:
                    continue
                A = np.array([[vx1,-vx2],[vy1,-vy2]])
                b = np.array([x2-x1, y2-y1])
                t,_ = np.linalg.solve(A, b)
                x,y = x1 + t*vx1, y1 + t*vy1
                if np.isfinite(x) and np.isfinite(y):
                    points.append([x,y])
        return np.array(points) if points else None

    @staticmethod
    def estimate_vanishing_points(intersections):
        if intersections is None or len(intersections) < 2:
            return None
        best_vp, best_in = None, 0
        for _ in range(RANSAC_ITERARATIONS):
            i,j = np.random.choice(len(intersections), 2, replace=False)
            samp = intersections[[i,j]].astype(np.float32)
            vx,vy,x0,y0 = cv2.fitLine(samp, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            # distance from all pts to this line
            d = np.abs((intersections[:,0]-x0)*vy - (intersections[:,1]-y0)*vx)
            inliers = d < RANSAC_INLIER_THRESHOLD
            cnt = inliers.sum()
            if cnt > best_in:
                best_in = cnt
                best_vp = intersections[inliers].mean(axis=0)
        return tuple(best_vp.astype(int)) if best_vp is not None else None



    # Callback function of the '/camera/image_raw' topic
    def frame_cb(self, msg):
        # Checking if initialization already done
        if  self.horizon_initialized:
            self.get_logger().info("Already initialized. Ignoring further frames.")
            self.get_logger().info("Node will now shut down.")
            self.destroy_subscription(self.subscription) 
            self.destroy_node()
            rclpy.shutdown()
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_frame.shape[:2]
            self.frame_count += 1

            # Debug check
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['original'], f"frame_{self.frame_count:04d}.jpg"), cv_frame)

            # Detect Charuco    
            found, corners, ids, detected_image = self.detect_charuco(cv_frame, self.charuco_board)
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['detect_charuco'], f"frame_{self.frame_count:04d}.jpg"), detected_image)
            if not found:
                self.get_logger().info("No Charuco detected")
                return
            
            
            # Detect Corners
            corner_image = aruco.drawDetectedCornersCharuco(cv_frame.copy(), corners, ids)
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['corners'], f"frame_{self.frame_count:04d}.jpg"),  corner_image)


            # Detecting lines using fixed-ID sets
            row_lines, column_lines = self.get_charuco_lines_fixed5X7(corners, ids)
            line_image = cv_frame.copy()
            for vx,vy,x0,y0 in row_lines + column_lines:
                p1 = (int(x0 - 1000*vx), int(y0 - 1000*vy))
                p2 = (int(x0 + 1000*vx), int(y0 + 1000*vy))
                cv2.line(line_image, p1, p2, (0,255,0), 2)
            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['lines'], f"frame_{self.frame_count:04d}.jpg"),  line_image)

            # Vanishing points along each board direction
            row_intersections  = self.set_intersections(row_lines)    # intersections among row-lines
            column_intersections  = self.set_intersections(column_lines)     # intersections among col-lines
            vp_row = self.estimate_vanishing_points(row_intersections)
            vp_col = self.estimate_vanishing_points(column_intersections)

            # Horizon connecting the two VPs
            final = cv_frame.copy()
        
            # draw VPs
            if vp_row: cv2.circle(final, vp_row, 8, (0,255,255), -1)
            if vp_col: cv2.circle(final, vp_col, 8, (255,0,255), -1)

            # if both VPs found, compute horizon endpoints at x=0 and x=w
            if vp_row and vp_col:
                (x1,y1),(x2,y2) = vp_row, vp_col
                # slope m and intercept b
                m = (y2 - y1) / float(x2 - x1)
                b = y1 - m*x1
                # at x=0 and x=w
                y_0 = int(b)
                y_w = int(m*width + b)
                cv2.line(final, (0,y_0), (width,y_w), (0,0,255), 2)
                self.get_logger().info()
                
                if self.debug:
                    cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['horizon'],f"frame_{self.frame_count:04d}.jpg"), final)
                array = Float64MultiArray()
                array.data = [0, y_0, width, y_w]
                self.publish_horizon.publish(array)
                self.get_logger().info("Published horizon line details")
                self.horizon_initialized = True
                self.destroy_subscription(self.subscription) 
                self.destroy_node()
                rclpy.shutdown()

            if self.debug:
                cv2.imwrite(os.path.join(MAIN_DEBUG_DIR, DEBUG_DIRS['horizon'],f"frame_{self.frame_count:04d}.jpg"), final)
                

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
