#!/usr/bin/env python3
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import traceback
from geometry_msgs.msg import Twist
import sys
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
import time 

class MainNode(Node):
    def __init__(self, node_name='main_node'):
    
        super().__init__(node_name)

        self.multi_thread_group = ReentrantCallbackGroup()
        self.cv_bridge = CvBridge()
        self.subscription_image = self.create_subscription(Image,'/camera/image_raw', self.main_cb,1,callback_group=self.multi_thread_group)
        self.publisher_cmd_vel= self.create_publisher(Twist,'/cmd_vel', 10)
        self.subscription_horizon_line = self.create_subscription(Float64MultiArray, '/horizon_line', self.horizon_cb,1,callback_group=self.multi_thread_group)
        self.publisher_processed = self.create_publisher(Image,'/final_display', 1)
        self.subscription_optical=self.create_subscription(Bool,'/stop_robot',self.of_cb,1,callback_group=self.multi_thread_group)
        self.subscription_stop=self.create_subscription(Bool,'/stop_sign',self.stop_sign_cb,1,callback_group=self.multi_thread_group)
        self.horizon_not_initialized = True
        
        self.vanishing_points = None
        self.y_0 = None 
        self.y_1 = None 
        self.y_2 = None 
        self.y_3 = None 
        self.frame_count = 0
        self.object_detected = False
        self.stop_sign_detected  = False
        self.camera_matrix = np.matrix([
            [610.78565932,   0.        , 154.50548085],
            [  0.        , 594.07200631, 127.60019182],
            [  0.        ,   0.        ,   1.        ]])
        self.no_aruco_detected = 0
        
    @staticmethod
    def horizon_line_drawer(image, vanishing_points):
        h, w = image.shape[:2]
        for (x_value, y_value) in vanishing_points:
            x_point = int(round(x_value))
            y_point = int(round(y_value))
            cv2.circle(image, (x_point, y_point), 5, (0, 0, 255), -1)
        
        if len(vanishing_points) == 1:
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

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0   
        msg.linear.y = 0.0  
        msg.linear.z = 0.0   
        msg.angular.x = 0.0  
        msg.angular.y = 0.0  
        msg.angular.z = 0.0  
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info("Stopping the TurtleBot")
        return None
    
    def aruco_move_robot(self,tvec,yaw):
        self.get_logger().info("aruco move")
        msg = Twist()
        # Forward and sideways distances (in camera frame)
        x = tvec[0][0]   # left-right (positive = right)
        z = tvec[0][2]   # forward distance (positive = forward)

        msg.linear.x = 0.1
        msg.angular.z = 0.0
        if z < 1.2:
            self.get_logger().info("aruco turn")
            yaw = yaw-0.255
            print(yaw)
            angular_k = 0.5
            msg.angular.z = -angular_k * yaw  # Rotate to align with marker long axis
            msg.angular.z = np.clip(msg.angular.z,-1,1)

        # Publish the command
        self.publisher_cmd_vel.publish(msg)

    def of_cb(self,msg):
        self.object_detected = msg.data
        return None


    def horizon_cb(self,msg):
        self.horizon_not_initialized = False  
        data = msg.data
        length    = len(data)//2
        self.vanishing_points  = [ (data[2*i], data[2*i+1]) for i in range(length)]
        self.get_logger().info(f"Received Horizon value")
        
        return None
    

    def stop_sign_cb(self,msg):
        self.stop_sign_detected = msg.data
        return None
   

    def main_cb(self,msg):
        self.frame_count = self.frame_count +1
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        dist_coeffs = np.zeros((5,))
        marker_length = 0.1

        if self.horizon_not_initialized:
            self.get_logger().info("Waiting for horizon finder to find horizon")
<<<<<<< HEAD
            # if self.stop_sign_detected:
            #     try :
            #         cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #         self.get_logger().info("Stop sign detected")
            #         self.stop_robot()  # To stop the robot
            #         label = "Stop sign detected"
            #         cv2.putText(cv_frame, label,(30, 30),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)    
            #         image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
            #         self.publisher_processed.publish(image_msg)
            #         return None
            #     except Exception as e:
            #         self.get_logger().error(f"Error in processing frame: {str(e)}")
            
            # elif not self.stop_sign_detected :
            #     cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #     image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
            #     self.publisher_processed.publish(image_msg)
                    
=======
      
>>>>>>> origin/zinobile
            if self.frame_count % 50 ==0:
                msg = Twist()
                msg.linear.x = 0.001
                msg.linear.y = 0.0  
                msg.linear.z = 0.0   
                msg.angular.x = 0.0  
                msg.angular.y = 0.0  
                msg.angular.z = 0.0   
                self.publisher_cmd_vel.publish(msg)
            return None
    
        try:

            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_frame_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
            height, width = cv_frame.shape[:2]

            # Drawing the horizon line
            cv_frame =self.horizon_line_drawer(cv_frame, self.vanishing_points)

            if self.object_detected:
                self.get_logger().info("Unsafe motion detected... stopping robot")
                label = "Unsafe - Object detected"
                color = (0, 0, 255)
                cv2.putText(cv_frame, label, (30, 100), cv2.FONT_HERSHEY_SIMPLEX,1.0,color,2, cv2.LINE_AA)

                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
                self.stop_robot()
<<<<<<< HEAD
                cv2.putText(cv_frame, label, (30, 100), cv2.FONT_HERSHEY_SIMPLEX,1.0,color,2, cv2.LINE_AA)
                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
=======
>>>>>>> origin/zinobile
                return None
            else:
                label = "Safe to Move - No object"
                color = (0, 255, 0)
                cv2.putText(cv_frame, label, (30, 100), cv2.FONT_HERSHEY_SIMPLEX,1.0,color,2, cv2.LINE_AA)
               

            #--------------------------------Stop sign logic start--------------------------------
            if self.stop_sign_detected:
                self.get_logger().info("Stop sign detected")
                self.stop_robot()  # To stop the robot
                label = "Stop sign detected"
                cv2.putText(cv_frame, label,(30, 30),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)    
                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame
            #--------------------------------Stop sign logic end--------------------------------
            




            #-----------------------------Regular path following logic start---------------------------

            aruco_corners, ids, rejected = cv2.aruco.detectMarkers(cv_frame_gray,aruco_dict,parameters=parameters)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_frame,aruco_corners,ids)
                closest_rvec = None
                closest_tvec = None
                centroid_y_old = 0

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, marker_length, self.camera_matrix, dist_coeffs)
                for i,(rvec,tvec) in enumerate(zip(rvecs,tvecs)):
                    z = tvec[0][2]  # Forward distance
                    corners = aruco_corners[i][0] 
                    centroid_x = int(np.mean(corners[:, 0]))
                    centroid_y = int(np.mean(corners[:, 1]))
                    if centroid_y > centroid_y_old:
                        centroid_y_old = centroid_y
                        closest_rvec = rvec
                        closest_tvec = tvec

                if closest_rvec is not None:

                    #cv2.drawFrameAxes(cv_frame,self.camera_matrix,dist_coeffs,rvec,tvec,0.05)
                    rotation_matrix,_ = cv2.Rodrigues(closest_rvec)
                    if sys.argv[1] == "x":

                        marker_x_axis = rotation_matrix[:,0]
                        x_axis_proj = np.array([marker_x_axis[0], 0, marker_x_axis[2]])
                        x_axis_proj /= np.linalg.norm(x_axis_proj)
                        yaw = np.arctan2(x_axis_proj[0], x_axis_proj[2])

                        # Define two 3D points: one at the origin of the marker, another along the x-axis direction
                        arrow_start_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)  # Marker origin
                        arrow_end_3d = np.array([[0.1, 0.0, 0.0]], dtype=np.float32)    # 10 cm along x-axis
                    
                    elif sys.argv[1] == "y":
                        marker_y_axis = rotation_matrix[:,1]
                        y_axis_proj = np.array([marker_y_axis[0], 0, marker_y_axis[2]])
                        y_axis_proj /= np.linalg.norm(y_axis_proj)
                        yaw = np.arctan2(y_axis_proj[0], y_axis_proj[2])
                        arrow_start_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)  # Marker origin
                        arrow_end_3d = np.array([[0.0, 0.1, 0.0]], dtype=np.float32)    # 10 cm along x-axis

                    # Project to 2D image plane
                    start_2d, _ = cv2.projectPoints(arrow_start_3d, closest_rvec, closest_tvec, self.camera_matrix, dist_coeffs)
                    end_2d, _ = cv2.projectPoints(arrow_end_3d, closest_rvec, closest_tvec, self.camera_matrix, dist_coeffs)

                    # Convert to integer pixel coordinates
                    start_pt = tuple(start_2d[0][0].astype(int))
                    end_pt = tuple(end_2d[0][0].astype(int))

                    # Draw the arrow on the image
                    cv2.arrowedLine(cv_frame, start_pt, end_pt, (0, 255, 0), 3, tipLength=0.3)
                    self.aruco_move_robot(closest_tvec,yaw)
                    self.no_aruco_detected = 0

            else:
                if self.no_aruco_detected > 50:
                    self.stop_robot()
                    
                else:
                    self.get_logger().info("no aruco detected")
                    tvec = [[2.0,2.0,2.0],[2.0,2.0,2.0]]
                    yaw = 0.0
                    self.aruco_move_robot(tvec,yaw)
                    self.no_aruco_detected += 1
            image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
            self.publisher_processed.publish(image_msg)
            #-----------------------------Regular path following logic end---------------------------

        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None