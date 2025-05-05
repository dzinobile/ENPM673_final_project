#!/usr/bin/env python3
import sys
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

from enpm673_module.enpm673_final_proj import *

class DirectionalMotion(Node):
    def __init__(self):
        super().__init__('DirectionalMotion')
        self.camera_matrix = np.matrix([
            [610.78565932,   0.        , 154.50548085],
            [  0.        , 594.07200631, 127.60019182],
            [  0.        ,   0.        ,   1.        ]])

        self.previous_z = 10000
        self.bridge = CvBridge()
        self.raw_image_sub = self.create_subscription(Image,'/camera/image_raw',self.raw_image_callback,10)
        self.publisher_cmd_vel= self.create_publisher(Twist,'/cmd_vel', 10)

    def move_robot(self,tvec,yaw):
        msg = Twist()
         # Extract forward and sideways distances (in camera frame)
        x = tvec[0][0]   # left-right (positive = right)
        z = tvec[0][2]   # forward distance (positive = forward)

        # Basic control gains (tune these)
        linear_k = 0.5
        #angular_k = 1.0

        # Desired linear and angular velocities
        msg.linear.x = linear_k * z  # Drive forward toward marker
        #msg.angular.z = -angular_k * yaw  # Rotate to align with marker long axis

        # Optional: limit max speed
        msg.linear.x = np.clip(msg.linear.x, -0.3, 0.3)
        #msg.angular.z = np.clip(msg.angular.z, -1.0, 1.0)

        # Publish the command
        self.publisher_cmd_vel.publish(msg)
        print(msg)

    def turn_robot

    def raw_image_callback(self,msg):
    #def raw_image_callback(self,msg):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        dist_coeffs = np.zeros((5,))
        marker_length = 0.1
        

        #camera_matrix = np.array([[527.09681904, 0, 319.50217002],
                                  #[0, 30.38209677, 239.49903363],
                                  #[0, 0, 1]], dtype=np.float32)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            aruco_corners, ids, rejected = cv2.aruco.detectMarkers(cv_image_gray,aruco_dict,parameters=parameters)

            if ids is not None and self.camera_matrix is not None:
                cv2.aruco.drawDetectedMarkers(cv_image,aruco_corners,ids)
                min_distance = float('inf')
                closest_rvec = None
                closest_tvec = None

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, marker_length, self.camera_matrix, dist_coeffs)
                for rvec,tvec in zip(rvecs,tvecs):
                    z = tvec[0][2]  # Forward distance
                    if z < min_distance:
                        min_distance = z
                        closest_rvec = rvec
                        closest_tvec = tvec
                if closest_rvec is not None:

                    #cv2.aruco.drawDetectedMarkers(cv_image,aruco_corners,ids)
                    cv2.drawFrameAxes(cv_image,self.camera_matrix,dist_coeffs,rvec,tvec,0.05)
                    rotation_matrix,_ = cv2.Rodrigues(closest_rvec)
                    marker_x_axis = rotation_matrix[:,0]
                    x_axis_proj = np.array([marker_x_axis[0], 0, marker_x_axis[2]])
                    x_axis_proj /= np.linalg.norm(x_axis_proj)
                    yaw = np.arctan2(x_axis_proj[0], x_axis_proj[2])
                    z = tvec[0][2]
                    if z < self.previous_z:
                        self.move_robot(closest_tvec,yaw)
                        self.previous_z = z


            
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = DirectionalMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()