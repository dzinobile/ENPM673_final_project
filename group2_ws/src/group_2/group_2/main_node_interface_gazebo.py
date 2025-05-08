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

class MainNode(Node):
    def __init__(self, node_name='main_node'):
    
        super().__init__(node_name)


        self.cv_bridge = CvBridge()
        self.subscription_image = self.create_subscription(Image,'/camera/image_raw', self.main_cb,10)
        self.publisher_cmd_vel= self.create_publisher(Twist,'/cmd_vel', 10)
        self.subscription_horizon_line = self.create_subscription(Float64MultiArray, '/horizon_line', self.horizon_cb,1)
        self.publisher_processed = self.create_publisher(Image,'/final_display', 10)
        self.model = torch.hub.load('yolov5','yolov5s', source='local')
        self.model.eval()
        self.horizon_not_initialized = True
        self.x_0 = None 
        self.y_0 = None
        self.x_w = None
        self.y_w = None
        self.object_detected = False
        self.camera_matrix = np.matrix([
            [610.78565932,   0.        , 154.50548085],
            [  0.        , 594.07200631, 127.60019182],
            [  0.        ,   0.        ,   1.        ]])
        self.no_aruco_detected = 0
        self.frame1 = None 
        self.frame2 = None
        self.smoothed_u = None 
        self.smoothed_v = None 
        self.alpha = 0.8
        
        self.publisher_flow_image = self.create_publisher(
            Image,
            '/flow_image',
            10
        )

        self.publisher_residual_image = self.create_publisher(
            Image, 
            '/residual_flow_image', 
            10
        )
        
        self.publisher_mask_image = self.create_publisher(
            Image, 
            '/flow_mask_image', 
            10
        )

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0   
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




    def horizon_cb(self,msg):
        self.horizon_not_initialized = False
        self.x_0, self.y_0, self.x_w, self.y_w = msg.data
        self.get_logger().info(f"Received Horizon value")
        return None
    

    def of_crop(self, gray, wpl=0.15, wph=0.85, hpl=.5, hph=1):
        H, W = gray.shape
        return gray[int((self.y_0+self.y_w)/2):int(hph*H), int(wpl*W):int(wph*W)]
    
    def of_convert_gray(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    def of_standardize(self, frame):
        return self.of_crop(self.of_convert_gray(frame))
    
    def of_polar(self, flow):
        return cv2.cartToPolar(flow[..., 0], flow[..., 1])

    def of_flow_image(self):
        mag, ang = cv2.cartToPolar(self.smoothed_u, self.smoothed_v)
        hsv = np.zeros((self.smoothed_u.shape[0], self.smoothed_u.shape[1], 3), dtype=np.uint8)
        hsv[..., 0] = ang * 180/np.pi/2 # 8-bit hue: [0, 360] deg -> [0, 180]
        hsv[..., 1] = 255
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        flow_image_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return flow_image_bgr
    
    def of_residual_image(self, flow):

        detected = False
        
        # 1. Convert to polar
        mag, ang = self.of_polar(flow)
        # 2. Subtract away the typical value 
        b = np.percentile(mag, 50) # Typical non-ego flow mag of frame
        #b = np.average(mag)
        mag_corr = mag - b 
        mag_corr[mag_corr < 0] = 0 
        
        thresh = 1.0 #Tuning parameter
        mask = (mag_corr > thresh)
        activated_vals = mag_corr[mask]
        if activated_vals.size:
            activated_average = np.average(activated_vals)
        else:
            activated_average = 0.0 
        
        percent_full = 100 * mask.mean()
        self.get_logger().info(f"Percent full: {percent_full: .2f}; Activated Average {activated_average: .2f}")

        if percent_full > 10:
            self.get_logger().info("Obstacle detected with obstacle flow")
            detected = True 

        mask_image_gray = (mask * 255).astype(np.uint8)

        # Visualization
        V = (mag_corr / np.max(mag_corr) * 255).astype(np.uint8)
        hsv = np.zeros((flow.shape[0], flow.shape[1], 3), dtype=np.uint8)
        hsv[..., 0] = mask * (ang * 180/np.pi/2).astype(np.uint8)
        hsv[..., 1] = mask * 255
        hsv[..., 2] = mask * V
        flow_image_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        #cv2.imshow("Residual image", flow_image_bgr)
        #cv2.imshow("Mask", (mask * 255).astype(np.uint8))
        
        return flow_image_bgr, mask_image_gray, detected
    
    def of_optical_flow(self):
        flow = cv2.calcOpticalFlowFarneback(
                self.frame1, self.frame2, None, .5, 3, 30, 3, 5, 1.2, 0
            ) 
        return flow
    
    def of_estimate_ego_motion(self):
        u, v = self.smoothed_u.flatten(), self.smoothed_v.flatten()
        u_est = np.median(u).astype(np.uint8)
        v_est = np.median(v).astype(np.uint8)
        return u_est, v_est



    def main_cb(self,msg):



        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        dist_coeffs = np.zeros((5,))
        marker_length = 0.1

        if self.horizon_not_initialized:
            self.get_logger().info("Waiting for horizon finder to find horizon")
            return None
        try:

            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_frame_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
            height, width = cv_frame.shape[:2]

            if self.frame1 is None: 
                self.frame1 = self.of_standardize(cv_frame)
                return None
            self.frame2 = self.of_standardize(cv_frame)


            # Drawing the horizon line
            cv2.line(cv_frame, (0,int(self.y_0)), (width,int(self.y_w)), (0,0,255), 2)


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
                image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                self.publisher_processed.publish(image_msg)
                return None  # Go to next frame
            #--------------------------------Stop sign logic end--------------------------------



            #-----------------------------Optical Flow logic start---------------------------------
            # Optical Flow code

            wpl, wph = 0.15, 0.85
            hpl, hph = 0.5, 1.0

            x1 = int(wpl * width)
            x2 = int(wph * width)
            y1 = int((self.y_0+self.y_w)/2)
            y2 = int(hph * height)


            flow = self.of_optical_flow()
            if self.smoothed_u is None:
                self.smoothed_u = flow[..., 0]
                self.smoothed_v = flow[..., 1]
            else:
                self.smoothed_u = self.alpha * self.smoothed_u + (1 - self.alpha) * flow[..., 0]
                self.smoothed_v = self.alpha * self.smoothed_v + (1 - self.alpha) * flow[..., 1]

            flow_image_bgr = self.of_flow_image()
            u_est, v_est = self.of_estimate_ego_motion()

            residual = np.copy(flow)
            residual[..., 0] -= u_est 
            residual[..., 1] -= v_est
            self.get_logger().info(f"DEBUG ego: u_est={u_est:.2f}, v_est={v_est:.2f}")
            
            flow_image_bgr, mask_image_gray, self.object_detected = self.of_residual_image(residual)

            cv2.rectangle(
                cv_frame,
                (x1, y1),
                (x2, y2),
                color=(255, 0, 0),
                thickness=2
            )

            self.frame1 = self.frame2

            if self.object_detected:
                label = "Unsafe"
                color = (0, 0, 255)
            else:
                label = "Safe"
                color = (0, 255, 0)
            
            cv2.putText(
                cv_frame, 
                label, 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                2, 
                cv2.LINE_AA
            )

            # Publish residual flow and mask





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
                    marker_x_axis = rotation_matrix[:,0]
                    x_axis_proj = np.array([marker_x_axis[0], 0, marker_x_axis[2]])
                    x_axis_proj /= np.linalg.norm(x_axis_proj)
                    yaw = np.arctan2(x_axis_proj[0], x_axis_proj[2])

                    # Define two 3D points: one at the origin of the marker, another along the x-axis direction
                    arrow_start_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)  # Marker origin
                    arrow_end_3d = np.array([[0.1, 0.0, 0.0]], dtype=np.float32)    # 10 cm along x-axis

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
            #-----------------------------Regular path following logic end---------------------------

            image_msg = self.cv_bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
            self.publisher_processed.publish(image_msg)
            
            flow_msg = self.cv_bridge.cv2_to_imgmsg(flow_image_bgr, encoding='bgr8')
            self.publisher_flow_image.publish(flow_msg)

            residual_flow_msg = self.cv_bridge.cv2_to_imgmsg(flow_image_bgr, encoding='bgr8')
            self.publisher_residual_image.publish(residual_flow_msg)

            mask_msg = self.cv_bridge.cv2_to_imgmsg(mask_image_gray, encoding='mono8')
            self.publisher_mask_image.publish(mask_msg)

        except Exception as e:
            self.get_logger().error(f"Error in processing frame: {str(e)}")
            self.get_logger().error(traceback.format_exc()) 

        return None


