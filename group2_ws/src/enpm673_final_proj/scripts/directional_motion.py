#!/usr/bin/env python3
import sys
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

from enpm673_module.enpm673_final_proj import *

class DirectionalMotion(Node):
    def __init__(self):
        super().__init__('DirectionalMotion')
        self.camera_matrix = None

        self.bridge = CvBridge()
        self.raw_image_sub = self.create_subscription(Image,'/camera/image_raw',self.raw_image_callback,10)
        #self.img_pub = self.create_publisher(Image,'/camera/processed_image',10)
        self.calibration_start = False
    c_frames = []

    

    def raw_image_callback(self,msg,calibration_frames=c_frames):
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
            if self.calibration_start:
                h,w,_ = cv_image.shape
                cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                print(len(calibration_frames))
                if len(calibration_frames) < 50:
                    calibration_frames.append(cv_image_gray)
                    # MOVE ROBOT LEFT AND RIGHT AS WELL
                elif len(calibration_frames) == 50:
                    calibration_frames.append(cv_image_gray)
                    criteria =  (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    objp = np.zeros((7*5,3), np.float32)
                    objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)
                    objpoints = []
                    imgpoints = []

                    for i in range(0,50):
                        c_img = calibration_frames[i].copy()
                        ret, corners = cv2.findChessboardCorners(c_img,(7,5),cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
                        if ret:
                            objpoints.append(objp)
                            #corners2 = cv2.cornerSubPix(c_img,corners,(5,5),(-1,-1),criteria)
                            #imgpoints.append(corners2)
                            imgpoints.append(corners)
                    print(len(objpoints))
                    print(len(imgpoints))
                    _, self.camera_matrix, _, _, _= cv2.calibrateCamera(objpoints, imgpoints, c_img.shape[::-1], None, None)
                    print(self.camera_matrix)
                
                else:


                    aruco_corners, ids, rejected = cv2.aruco.detectMarkers(cv_image_gray,aruco_dict,parameters=parameters)

                    if ids is not None and self.camera_matrix is not None:
                        cv2.aruco.drawDetectedMarkers(cv_image,aruco_corners,ids)

                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, marker_length, self.camera_matrix, dist_coeffs)
                        for rvec,tvec in zip(rvecs,tvecs):
                            #cv2.aruco.drawDetectedMarkers(cv_image,aruco_corners,ids)
                            cv2.drawFrameAxes(cv_image,self.camera_matrix,dist_coeffs,rvec,tvec,0.05)
                            rotation_matrix,_ = cv2.Rodrigues(rvec)
                            print("Rotation Vector:\n",rvec)
                            print("Translation vector:\n",tvec)
                            print("Rotation matrix:\n", rotation_matrix)

            

            
            cv2.imshow("Camera View", cv_image)
            key = cv2.waitKey(1)  & 0xFF
            if key == ord("w"):
                self.calibration_start = True
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