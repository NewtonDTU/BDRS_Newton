import cv2
import numpy as np
import time as t
from datetime import datetime
from setproctitle import setproctitle

# robot modules
from spose import pose
from sir import ir
from srobot import robot
from sedge import edge
from sgpio import gpio
from uservice import service
from scripts.utils import driveTurn, drive_distance
from scripts.findAruco import find_aruco_markers
import signal
import sys

# --- Ctrl+C handler ---
stop_program = False
def shutdown_handler(sig, frame):
    global stop_program
    print("\nCtrl+C pressed! Stopping...")
    stop_program = True

signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)

# --- Camera setup ---
cam = cv2.VideoCapture('http://10.197.216.163:7123/stream.mjpg')

# --- Setup robot ---
setproctitle("findAruco")
service.setup('localhost')
edge.lineControl(0, True)
pose.setup()

# --- Main loop ---
state = 0
marker_pos_ee = None

start_find_aruco = True
find_aruco_state = 99
Kp_turn = 0.5
find_debouncer = 0
lost_debouncer = 0
found_marker = False
lost_marker = False
debounce_threshold = 1

try:
    while not stop_program:
        ret, frame = cam.read()
        if not ret:
            t.sleep(0.05)
            continue

        # detect marker
        marker_pos_ee, frame = find_aruco_markers(frame, expected_marker_id=20)
        if marker_pos_ee is not None:
            if find_debouncer > debounce_threshold:
                turn_angle = -np.arctan2(marker_pos_ee[0], marker_pos_ee[1])
                distance_to_marker = marker_pos_ee[2]
                found_marker = True
                lost_marker = False
            lost_debouncer = 0
            find_debouncer += 1
            lost_marker = False
        else:
            if lost_debouncer > debounce_threshold:
                lost_marker = True
                found_marker = False
                turn_angle = 0
                distance_to_marker = 0
            lost_debouncer += 1
            find_debouncer = 0
            found_marker = False

        if start_find_aruco:

            print (f"State: {find_aruco_state}")

            if find_aruco_state == 0:
                pose.tripBreset()
                print(f"Turning robot by {np.rad2deg(turn_angle):.2f} degrees to face marker...")
                find_aruco_state = 1
            
            elif find_aruco_state == 1:
                if lost_marker == True:
                    find_aruco_state = 99
                angular_speed = Kp_turn * turn_angle
                angular_speed = np.clip(angular_speed, -0.5, 0.5)
                service.send("robobot/cmd/ti", "rc 0.0 " + str(round(angular_speed, 2)))
                if abs(turn_angle) < np.deg2rad(2) or pose.tripBtimePassed() > 20:
                    print(f"Triggered by angle {np.rad2deg(turn_angle):.2f} degrees or timeout {pose.tripBtimePassed():.2f} seconds")
                    service.send("robobot/cmd/ti",f"rc 0.0 0.0")
                    find_aruco_state = 2
            
            elif find_aruco_state == 2:
                pose.tripBreset()
                print(f"Driving towards marker, distance: {distance_to_marker:.2f} m")
                find_aruco_state = 3

            elif find_aruco_state == 3:
                
                if distance_to_marker < 0.45:
                    find_aruco_state = 4
                    marker_close_time = t.time()
                else:
                    speed = 0.3
                service.send("robobot/cmd/ti", "rc " + str(round(speed, 2)) + " 0.0")

            elif find_aruco_state == 4:
                speed = 0.2
                service.send("robobot/cmd/ti", "rc " + str(round(speed, 2)) + " 0.0")
                if (t.time() - marker_close_time > 1.5) or (pose.tripBtimePassed()) > 20: # Presumably 40 cm is done in 2 seconds if the speed is 0.2 m/s
                    service.send("robobot/cmd/ti","rc 0.0 0.0")
                    find_aruco_state = 5

            elif find_aruco_state == 5:
                print("Reached marker!")
                find_aruco_state = 99
                start_find_aruco = False

            elif find_aruco_state == 99:
                service.send("robobot/cmd/ti",f"rc 0.0 0.2")
                if found_marker and service.connected:
                    service.send("robobot/cmd/ti",f"rc 0.0 0.0")
                    print(f"Marker position in EE frame:\n{marker_pos_ee}")
                    find_aruco_state = 0

except KeyboardInterrupt:
    print("\nKeyboard interrupt received, exiting...")

finally:
    cam.release()
    service.terminate()
    print("Shutdown complete")
    sys.exit(0)