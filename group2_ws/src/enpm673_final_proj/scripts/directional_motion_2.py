#!/usr/bin/env python3
import sys
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

from enpm673_module.enpm673_final_proj import *

class DirectionalMotion(Node):
    def __init__(self):
        super().__init__('DirectionalMotion')
        self.camera_matrix = np.matrix([
            [610.78565932,   0.        , 154.50548085],
            [  0.        , 594.07200631, 127.60019182],
            [  0.        ,   0.        ,   1.        ]])

        self.previous_z = 10000
        self.previous_yaw = 0
        self.bridge = CvBridge()
        self.raw_image_sub = self.create_subscription(Image,'/camera/image_raw',self.raw_image_callback,2)
        self.publisher_cmd_vel= self.create_publisher(Twist,'/cmd_vel', 10)
        self.iterations = 0


    def move_robot(self,tvec,yaw):
        msg = Twist()
         # Extract forward and sideways distances (in camera frame)
        x = tvec[0][0]   # left-right (positive = right)
        z = tvec[0][2]   # forward distance (positive = forward)
        linear_k = 0.1
        msg.linear.x = linear_k * z
        msg.linear.x = np.clip(msg.linear.x, -0.1, 0.1)
        yaw = yaw-0.03
        print(yaw)
        angular_k = 0.5
        msg.angular.z = -angular_k * yaw  # Rotate to align with marker long axis
        msg.angular.z = np.clip(msg.angular.z,-0.5,0.5)
        # if z > 1.5:
        #     a = np.rad2deg(np.arctan(x/z))+25
        #     if abs(a) > 2:
        #             angular_k = 0.5
        #             msg.angular.z = -angular_k*a
        #             msg.angular.z = np.clip(msg.angular.z, -0.3, 0.3)
        #             msg.linear.x = 0.0
        #     else:
        #         linear_k = 0.1
        #         msg.linear.x = linear_k * z
        #         msg.linear.x = np.clip(msg.linear.x, -0.1, 0.1)
        #         msg.angular.z = 0.0
        # else:
        #     yaw = yaw+0.6
        #     print(yaw)
        #     msg.linear.x = 0.001
        #     angular_k = 0.5
        #     msg.angular.z = -angular_k * yaw  # Rotate to align with marker long axis
        #     msg.angular.z = np.clip(msg.angular.z,-0.5,0.5)
        
        # Basic control gains (tune these)
        #linear_k = 0.1
        #msg.linear.x = linear_k * z
        #msg.linear.x = np.clip(msg.linear.x, -0.1, 0.1)


        # Desired linear and angular velocities
          # Drive forward toward marker
        # if z > 1:
        #     angular_k = 0.1
        #     msg.angular.z = -angular_k*a
        #     msg.angular.z = np.clip(msg.angular.z, -0.1, 0.1)
        # else:
        #     angular_k = 0.5
        #     msg.angular.z = -angular_k * yaw  # Rotate to align with marker long axis
        #     msg.angular.z = np.clip(msg.angular.z,-0.5,0.5)

        # Optional: limit max speed
        
        

        # Publish the command
        self.publisher_cmd_vel.publish(msg)
        

    def turn_robot(self,yaw):
        print("turn robot")
        msg= Twist()
        msg.angular.z = -0.5* yaw  # Rotate to align with marker long axis
        self.publisher_cmd_vel.publish(msg)
        time.sleep(2)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher_cmd_vel.publish(msg)

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
                    self.move_robot(closest_tvec,yaw)
                    # print(z)
                    # if z < self.previous_z+0.5:
                    #     self.move_robot(closest_tvec,yaw)
                    #     self.previous_z = z
                    #     self.previous_yaw = yaw
                    # else:
                    #     self.stop_robot()
                    #     self.turn_robot(self.previous_yaw)
                    #     self.previous_z = z
                    #     self.previous_yaw = yaw




            
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