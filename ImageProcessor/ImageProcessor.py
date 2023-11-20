import cv2
import cv2.aruco as aruco
import numpy as np
 
class ArucoProcessor:
    def __init__(self, camera_matrix, dist_coeff, HFOV=90, marker_size=0.25):
        self.camera_matrix = camera_matrix
        self.dist_coeff = dist_coeff
        self.HFOV = HFOV
        self.marker_size = marker_size
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()

    def process_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        frame_center = (gray.shape[1] / 2, gray.shape[0] / 2)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(image, corners, ids)
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeff)

            for rvec, tvec in zip(rvecs, tvecs):
                image = cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeff, rvec, tvec, self.marker_size / 2)

            pad_info = self.locate_pads(corners, ids, tvecs, frame_center)
            xyz_to_pads = self.get_xyz_to_pads(rvecs, tvecs)

            return pad_info, xyz_to_pads
        return [], []

    def locate_pads(self, corners, ids, tvecs, frame_center):
        pad_info = []
        for i, (corner, tvec) in enumerate(zip(corners, tvecs)):
            center = corner[0].mean(axis=0)
            x_diff = center[0] - frame_center[0]
            frame_width = frame_center[0] * 2
            angle_deg = (x_diff / frame_width) * self.HFOV / 2

            # Calculate the Euclidean distance
            distance = np.linalg.norm(tvec)
            
            pad_info.append((ids[i][0], angle_deg, distance))
        return pad_info

    def get_xyz_to_pads(self, rvecs, tvecs):
        xyz_to_pads = []
        for rvec, tvec in zip(rvecs, tvecs):
            R_ct, _ = cv2.Rodrigues(rvec)
            R_tc = R_ct.T
            camera_position = -R_tc @ tvec.reshape((3, 1))
            xyz_to_pads.append(camera_position.ravel())
        return xyz_to_pads
    
    def get_xyz_to_pads_down(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        xyz_to_pads = []
        if ids is not None:
            aruco.drawDetectedMarkers(image, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeff)
            for rvec, tvec in zip(rvecs, tvecs):
                R_ct, _ = cv2.Rodrigues(rvec)
                R_tc = R_ct.T
                camera_position = -R_tc @ tvec.reshape((3, 1))
                xyz = camera_position.ravel()

                # Adjusting XYZ coordinates based on the drone's perspective
                # Swap X and Y, and change the sign of Y to reflect the image-to-drone transformation
                adjusted_xyz = np.array([-xyz[1], -xyz[0], xyz[2]])

                xyz_to_pads.append(adjusted_xyz)
        return xyz_to_pads
    
    
    
    
# Some testing
"""def main():
    # Load previously saved calibration data
    calibration_data = np.load('TestingStuff/calibration_data.npz')
    camera_matrix = calibration_data['camera_matrix']
    dist_coeff = calibration_data['dist_coeff']

    # Initialize ArucoProcessor
    aruco_processor = ArucoProcessor(camera_matrix, dist_coeff)

    # Start video capture
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        pad_info, xyz_to_pads = aruco_processor.process_image(frame)

        for info, xyz in zip(pad_info, xyz_to_pads):
            distance_to_pad = np.linalg.norm(xyz)
            print(f"Pad ID: {info[0]}, Degrees from Center: {info[1]}, Distance to Pad: {distance_to_pad}, XYZ to Pad: {xyz}")


        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()"""
