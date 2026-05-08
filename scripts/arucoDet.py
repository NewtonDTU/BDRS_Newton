import cv2
import numpy as np

"""
input: frame (BGR image), camera_matrix, dist_coeffs, marker_size (in meters)
 - camera matrix and distortion coefficients can be obtained from camera calibration (e.g. using calib.py or if
you already have them from previous calibration, you can load them from a calibration.npz file)
 - marker_size changes with the physical size of the Aruco marker you are using (e.g. 0.05 for 5 cm markers)

return values:
 - rvec = [0.1, 0.2, 0.3] -> marker is rotated 0.1 rad around x, 0.2 rad around y, 0.3 rad around z
 - tvec = [0.1, -0.05, 0.5] -> marker is 10 cm right, 5 cm up, 50 cm away
 - distance -> in meters, calculated as sqrt(x^2 + y^2 + z^2)
 - marker_id -> integer ID of the detected marker
"""
def detectArucoMarkers(frame, camera_matrix, dist_coeffs, marker_size=0.05):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    detections = []

    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            img_points = marker_corners.reshape((4,2)).astype(np.float32)

            obj_points = np.array([
                [-marker_size/2, marker_size/2, 0],
                [ marker_size/2, marker_size/2, 0],
                [ marker_size/2,-marker_size/2, 0],
                [-marker_size/2,-marker_size/2, 0]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                camera_matrix,
                dist_coeffs
            )

            if success:
                distance = float(np.linalg.norm(tvec))
                detections.append((rvec, tvec, distance, int(marker_id), img_points))
                
        return detections

    return None