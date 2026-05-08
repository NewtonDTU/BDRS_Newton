import cv2
import numpy as np
import time
from scripts.arucoDet import detectArucoMarkers
from scripts.utils import driveTurn, drive_distance
#import sys
#import threading
import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from sedge import edge
from sgpio import gpio
from uservice import service

ID_SIZE_DICTIONARY = {
    10: 0.1,
    11: 0.1,
    12: 0.1,
    13: 0.1,
    14: 0.1,
    15: 0.1,
    16: 0.1,
    17: 0.1,
    5: 0.035,
    20: 0.035,
    53: 0.035,
    25: 0.1
}

def rvec_tvec_to_matrix(rvec, tvec):

    R, _ = cv2.Rodrigues(rvec)

    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = tvec.flatten()

    return T

def find_aruco_markers(frame, expected_marker_id=0):

    # ---- Load camera calibration ----
    data = np.load("calibration.npz")
    camera_matrix = data["cameraMatrix"]
    dist_coeffs = data["distCoeffs"]

    
    marker_size = ID_SIZE_DICTIONARY.get(expected_marker_id, 0)

    theta_deg = -25
    theta = np.deg2rad(theta_deg)


    T_cam_ee = np.array([
        [1,     0,              0,          0.00],
        [0,np.cos(theta), -np.sin(theta),   0.19],
        [0,np.sin(theta),  np.cos(theta),   0.04],
        [0,     0,              0,            1]
        ])

    # frame = cv2.undistort(frame, camera_matrix, dist_coeffs) # Do NOT undistort passed to detectMarkers!

    detections = detectArucoMarkers(frame, camera_matrix, dist_coeffs, marker_size)

    if detections is None:
        #print("Expected marker ID {} not found in detections.".format(expected_marker_id))
        return None, frame

    detection = next((d for d in detections if d[3] == expected_marker_id), None)

    if detection is None:
        #print("Expected marker ID {} not found in detections.".format(expected_marker_id))
        return None, frame

    rvec, tvec, distance, marker_id, img_points = detection

    pts = img_points.astype(int)

    # Draw marker bounding box
    cv2.polylines(frame, [pts], True, (0,255,0), 2)

    # Draw coordinate axes
    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

    #distance in end-effector frame

    T_marker_cam = rvec_tvec_to_matrix(rvec, tvec)
    T_marker_ee = T_cam_ee @ T_marker_cam
    marker_pos_ee = T_marker_ee[:3,3]
    distance = np.linalg.norm(marker_pos_ee)

    cv2.putText(
        frame,
        f"ID:{marker_id} Dist:{distance:.2f}m "
        f"X:{marker_pos_ee[0]:.2f} Y:{marker_pos_ee[1]:.2f} Z:{marker_pos_ee[2]:.2f}",
        (int(pts[0][0]-50), int(pts[0][1]) - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0,255,0),
        1
    )

    print("X:{:.2f}m Y:{:.2f}m Z:{:.2f}m".format(marker_pos_ee[0], marker_pos_ee[1], marker_pos_ee[2]))

    return marker_pos_ee, frame
