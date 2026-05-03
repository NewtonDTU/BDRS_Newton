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
ret, frame = cam.read()
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('aruco.avi', fourcc, 20.0, (frame.shape[1], frame.shape[0]))

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
        marker_pos_ee, frame = find_aruco_markers(frame, expected_marker_id=11)
        print(f"Marker distance: {marker_pos_ee[2]:.3f} m" if marker_pos_ee is not None else "Marker not detected")

        out.write(frame)
        
        

except KeyboardInterrupt:
    print("\nKeyboard interrupt received, exiting...")

finally:
    t.sleep(0.5)
    out.release()
    cam.release()
    service.terminate()
    print("Shutdown complete")
    sys.exit(0)