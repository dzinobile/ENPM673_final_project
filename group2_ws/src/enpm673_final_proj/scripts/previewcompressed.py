import cv2
import numpy as np

camera_matrix = np.matrix([
        [610.78565932,   0.        , 154.50548085],
        [  0.        , 594.07200631, 127.60019182],
        [  0.        ,   0.        ,   1.        ]])


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
parameters = cv2.aruco.DetectorParameters()
dist_coeffs = np.zeros((5,))
marker_length = 0.2

capture = cv2.VideoCapture('previewcompressed.mp4')
while True:
    ret,frame = capture.read()

    if not ret:
        break
    
    frame_grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_corners, ids, rejected = cv2.aruco.detectMarkers(frame_grey,aruco_dict,parameters=parameters)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame,aruco_corners,ids)
        closest_rvec = None
        closest_tvec = None
        centroid_y_old = 0
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, marker_length, camera_matrix, dist_coeffs)
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

                cv2.drawFrameAxes(frame,camera_matrix,dist_coeffs,rvec,tvec,0.05)
                rotation_matrix,_ = cv2.Rodrigues(closest_rvec)
                marker_x_axis = rotation_matrix[:,0]
                x_axis_proj = np.array([marker_x_axis[0], 0, marker_x_axis[2]])
                x_axis_proj /= np.linalg.norm(x_axis_proj)
                yaw = np.arctan2(x_axis_proj[0], x_axis_proj[2])
                z = tvec[0][2]
                print(z)
    cv2.imshow("frame",frame)
    cv2.waitKey(1)

cv2.destroyAllWindows()