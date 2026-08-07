import numpy as np
import cv2 as cv
import os

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Lists to store calibration results from each image
all_objpoints = []  # 3D points in real-world space
all_imgpoints = []  # 2D points in image plane.
all_matrices = []   # Camera matrices
all_coefficients = []  # Distortion coefficients
all_images = []  # Captured images

cap = cv.VideoCapture(2) 
cap.set(cv.CAP_PROP_AUTOFOCUS,0)

image_count = 0
max_images = 100
calibration_started = False

while True:
    ret, img = cap.read()
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    if calibration_started:
        ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

        if ret:
            objpoints = np.zeros((6*7, 3), np.float32)
            objpoints[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
            all_objpoints.append(objpoints)
            
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            all_imgpoints.append(corners2)
            
            cv.drawChessboardCorners(img, (7, 6), corners2, ret)

            image_count += 1

            image_filename = f'camera_calibration/captured_image_{image_count}.jpg'
            cv.imwrite(image_filename, img)
            all_images.append(image_filename)

            if image_count >= max_images:  

                ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(all_objpoints, all_imgpoints, gray.shape[::-1], None, None)

                mean_matrix = mtx
                mean_coefficients = dist

                np.savetxt('camera_calibration/mean_camera_matrix.txt', mean_matrix)
                np.savetxt('camera_calibration/mean_distortion_coefficients.txt', mean_coefficients)

                print("Mean Camera Matrix:")
                print(mean_matrix)
                print("\nMean Distortion Coefficients:")
                print(mean_coefficients)

                image_count = 0
                all_objpoints = []
                all_imgpoints = []
                all_matrices = []
                all_coefficients = []

                calibration_started = False

    cv.imshow('img', img)

    key = cv.waitKey(1)
    if key == 27:  # Press 'Esc' to exit
        break
    elif key == ord(' '):  # Press spacebar to start capturing and initiate calibration
        print("Calibration Started! Capture images...")
        calibration_started = True

# Release the camera
cap.release()
cv.destroyAllWindows()