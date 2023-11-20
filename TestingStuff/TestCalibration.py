import numpy as np
import cv2
import glob

from DroneInterface import DroneInterface

drone = DroneInterface.Drone()

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on the real chessboard size
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all images
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane

# Start the webcam capture
# cap = cv2.VideoCapture(0)

# Count of valid images taken
valid_images = 0

while True:
    img = drone.get_image()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        # Refine the corner positions
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (9,6), corners2, ret)

        valid_images += 1
        print(f"Found corners in {valid_images} images")

    cv2.imshow('img', img)
    
    # Press 'q' to close the window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam
# cap.release()
cv2.destroyAllWindows()

# Ensure that at least one image was processed
if len(objpoints) > 0:
    # Camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Save the camera matrix and distortion coefficients
    np.savez('drone_front.npz', ret=ret, camera_matrix=mtx, dist_coeff=dist, rvecs=rvecs, tvecs=tvecs)
    print("Calibration successful, data saved.")
else:
    print("Calibration was not successful. No images were captured.")
