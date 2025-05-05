from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import random
from sensor_msgs.msg import Image
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
        self.subscription_raw_image = self.create_subscription(Image,topic_prefix+'/oakd/rgb/preview/image_raw', self.main_cb,10)
        self.publisher_cmd_vel= self.create_publisher(TwistStamped, topic_prefix+'/cmd_vel', 10)
        self.subscription_horizon_y = self.create_subscription(Int32,topic_prefix+'/horizon_y', self.horizon_cb,10)
        self.publisher_processed = self.create_publisher(Image, topic_prefix+'/final_display', 10)
        self.model = torch.hub.load('yolov5','yolov5s', source='local')
        self.model.eval()
        self.horizon_not_initialized = True
        self.horizon_y = None
        self.object_detected = False
        self.camera_matrix = np.matrix([
            [610.78565932,   0.        , 154.50548085],
            [  0.        , 594.07200631, 127.60019182],
            [  0.        ,   0.        ,   1.        ]])

    def stop_robot(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0   
        msg.twist.angular.z = 0.0  
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info("Stopping the TurtleBot")
        return None
    
    def move_robot(self,tvec,yaw):
        msg = TwistStamped()
         # Extract forward and sideways distances (in camera frame)
        x = tvec[0][0]   # left-right (positive = right)
        z = tvec[0][2]   # forward distance (positive = forward)

        # Basic control gains (tune these)
        linear_k = 0.5
        angular_k = 1.0

        # Desired linear and angular velocities
        msg.twist.linear.x = linear_k * z  # Drive forward toward marker
        msg.twist.angular.z = -angular_k * yaw  # Rotate to align with marker long axis

        # Optional: limit max speed
        msg.twist.linear.x = np.clip(msg.twist.linear.x, -0.3, 0.3)
        msg.twist.angular.z = np.clip(msg.twist.angular.z, -1.0, 1.0)

        # Publish the command
        self.publisher_cmd_vel.publish(msg)



    def horizon_cb(self,msg):
        self.horizon_not_initialized = False
        self.horizon_y = msg.data
        self.get_logger().info(f"Received Horizon value : {self.horizon_y }")
        return None
    
    
    
    def main_cb(self,msg):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        dist_coeffs = np.zeros((5,))
        marker_length = 0.1

        if self.horizon_not_initialized or self.horizon_y is None:
            self.get_logger().info("Waiting for horizon finder to find horizon")
            return None
        try:
            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
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
                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame
            #--------------------------------Stop sign logic end--------------------------------



            #-----------------------------Optical Flow logic start---------------------------------
            # Optical Flow code
            if self.object_detected:
                self.get_logger().info("Object detected")
                self.stop_robot()  # To stop the robot
                # Final display being published
                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame
            #-----------------------------Optical Flow logic end-------------------------------------



            #-----------------------------Regular path following logic start---------------------------

            aruco_corners, ids, rejected = cv2.aruco.detectMarkers(cv_frame_gray,aruco_dict,parameters=parameters)

            if ids is not None and self.camera_matrix is not None:
                cv2.aruco.drawDetectedMarkers(cv_frame,aruco_corners,ids)
                min_distance = float('inf')
                closest_rvec = None
                closest_tvec = None

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, marker_length, self.camera_matrix, dist_coeffs)
                for rvec,tvec in zip(rvecs,tvecs): #tvec is translation vector [x,y,z] in camera coordinates
                    z = tvec[0][2]  # Forward distance
                    if z < min_distance:
                        min_distance = z
                        closest_rvec = rvec
                        closest_tvec = tvec
                if closest_rvec is not None:
                    cv2.drawFrameAxes(cv_frame,self.camera_matrix,dist_coeffs,rvec,tvec,0.05)
                    rotation_matrix,_ = cv2.Rodrigues(closest_rvec)
                    marker_x_axis = rotation_matrix[:,0]
                    x_axis_proj = np.array([marker_x_axis[0], 0, marker_x_axis[2]])
                    x_axis_proj /= np.linalg.norm(x_axis_proj)
                    yaw = np.arctan2(x_axis_proj[0], x_axis_proj[2])
                    self.move_robot(closest_tvec,yaw)
            #-----------------------------Regular path following logic end---------------------------

            # Final display being published
            image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
            self.publisher_processed.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None


