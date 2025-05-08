from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import random
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import torch
import traceback
from geometry_msgs.msg import TwistStamped
import sys
import numpy as np

class MainNode(Node):
    def __init__(self, node_name='main_node'):
    
        super().__init__(node_name)
        which_robot = int(sys.argv[1])
        if which_robot == 1:
            topic_prefix = '/tb4_1'
        elif which_robot == 2:
            topic_prefix = '/tb4_2'


        self.cv_bridge = CvBridge()
        self.subscription_image_compressed = self.create_subscription(CompressedImage,topic_prefix+'/oakd/rgb/image_raw/compressed', lambda msg: self.main_cb(msg, "raw_compressed"),10)
        self.subscription_image_variable = self.create_subscription(CompressedImage,topic_prefix+image_topic, lambda msg: self.main_cb(msg, "variable"),10)
        self.publisher_cmd_vel= self.create_publisher(TwistStamped, topic_prefix+'/cmd_vel', 10)
        self.subscription_horizon_y = self.create_subscription(Int32,topic_prefix+'/horizon_y', self.horizon_cb,10)
        self.publisher_processed = self.create_publisher(CompressedImage, topic_prefix+'/final_display', 10)
        self.model = torch.hub.load('yolov5','yolov5s', source='local')
        self.model.eval()
        self.horizon_not_initialized = True
        self.horizon_y = None
        self.object_detected = False
        self.camera_matrix = np.matrix([
            [610.78565932,   0.        , 154.50548085],
            [  0.        , 594.07200631, 127.60019182],
            [  0.        ,   0.        ,   1.        ]])
        self.no_aruco_detected = 0

    def stop_robot(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0   
        msg.twist.angular.z = 0.0  
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info("Stopping the TurtleBot")
        return None
    
    def aruco_move_robot(self,tvec,yaw):
        msg = TwistStamped()
        # Forward and sideways distances (in camera frame)
        x = tvec[0][0]   # left-right (positive = right)
        z = tvec[0][2]   # forward distance (positive = forward)

        msg.twist.linear.x = 0.1
        msg.twist.angular.z = 0.0
        if z < 1:
            yaw = yaw-0.255
            print(yaw)
            angular_k = 0.5
            msg.twist.angular.z = -angular_k * yaw  # Rotate to align with marker long axis
            msg.twist.angular.z = np.clip(msg.twist.angular.z,-1,1)

        # Publish the command
        self.publisher_cmd_vel.publish(msg)



    def horizon_cb(self,msg):
        self.horizon_not_initialized = False
        self.horizon_y = msg.data
        self.get_logger().info(f"Received Horizon value : {self.horizon_y }")
        return None
    
    
    
    def main_cb(self,msg,source):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        parameters = cv2.aruco.DetectorParameters()
        dist_coeffs = np.zeros((5,))
        marker_length = 0.2

        if self.horizon_not_initialized or self.horizon_y is None:
            self.get_logger().info("Waiting for horizon finder to find horizon")
            return None
        try:
            
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_frame_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
            height, width = cv_frame.shape[:2]
            # Drawing the horizon line
            cv2.line(cv_frame, (0, self.horizon_y), (width-1, self.horizon_y), (0,255,0), 2)



            #--------------------------------Stop sign logic start--------------------------------
            
            #  Run inference
            results = self.model(cv_frame)
            labels = results.names
            detections = results.pred[0]
            stop_sign_detected = False
            for *box, conf, cls in detections:
                label = labels[int(cls)]
                if label.lower() == "stop sign":
                    stop_sign_detected = True
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(cv_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(cv_frame, f"{label} ({conf:.2f})", (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)      
            
            if stop_sign_detected:
                self.get_logger().info("Stop sign detected")
                self.stop_robot()  # To stop the robot
                # Final display being published
                image_msg = CompressedImage()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.format = 'jpeg'
                _, buffer = cv2.imencode('.jpg', cv_frame)
                image_msg.data = np.array(buffer).tobytes()
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame
            #--------------------------------Stop sign logic end--------------------------------



            #-----------------------------Optical Flow logic start---------------------------------
            # Optical Flow code
            if self.object_detected:
                self.get_logger().info("Object detected")
                self.stop_robot()  # To stop the robot
                # Final display being published
                image_msg = CompressedImage()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.format = 'jpeg'
                _, buffer = cv2.imencode('.jpg', cv_frame)
                image_msg.data = np.array(buffer).tobytes()
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame
            #-----------------------------Optical Flow logic end-------------------------------------



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

                    cv2.drawFrameAxes(cv_frame,self.camera_matrix,dist_coeffs,rvec,tvec,0.05)
                    rotation_matrix,_ = cv2.Rodrigues(closest_rvec)
                    marker_x_axis = rotation_matrix[:,0]
                    x_axis_proj = np.array([marker_x_axis[0], 0, marker_x_axis[2]])
                    x_axis_proj /= np.linalg.norm(x_axis_proj)
                    yaw = np.arctan2(x_axis_proj[0], x_axis_proj[2])
                    self.aruco_move_robot(closest_tvec,yaw)
                    self.no_aruco_detected = 0

            else:
                if self.no_aruco_detected > 50:
                    self.stop_robot()
                    
                else:
                    tvec = [[2.0,2.0,2.0],[2.0,2.0,2.0]]
                    yaw = 0.0
                    self.aruco_move_robot(tvec,yaw)
                    self.no_aruco_detected += 1
            #-----------------------------Regular path following logic end---------------------------

            # Final display being published
            image_msg = CompressedImage()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.format = 'jpeg'
            _, buffer = cv2.imencode('.jpg', cv_frame)
            image_msg.data = np.array(buffer).tobytes()
            self.publisher_processed.publish(image_msg)

            cv2.imshow('display',cv_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None


