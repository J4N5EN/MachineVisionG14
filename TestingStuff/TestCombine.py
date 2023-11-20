import cv2
import cv2.aruco as aruco
import numpy as np
from DroneInterface import DroneInterface

drone = DroneInterface.Drone()

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# Prepare object points based on the real chessboard size
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)


def main():
    # Load previously saved calibration data
    calibration_data = np.load('drone_front.npz')
    camera_matrix = calibration_data['camera_matrix']
    dist_coeff = calibration_data['dist_coeff']

    

    # Load the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    while True:
        # Capture frame-by-frame
        frame = drone.get_image()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Check if at least one marker was detected
        if ids is not None:
            # Draw detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimate the pose of each marker
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.1, camera_matrix, dist_coeff)
            
            for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                # Calculate camera position in world coordinates
                R_ct, _ = cv2.Rodrigues(rvec)
                R_tc = R_ct.T
                camera_position = -R_tc @ tvec.reshape((3, 1))
                print(f"Camera Position relative to marker {ids[i][0]}: {camera_position.ravel()}")

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()