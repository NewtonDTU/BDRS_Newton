
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

# --- Camera setup ---
cam = cv2.VideoCapture('http://10.197.216.163:7123/stream.mjpg')

num_of_frames = 0
t_prev = t.time()

while num_of_frames < 15:
    ret, frame = cam.read()
    if not ret:
        t.sleep(0.05)
        continue
    
    if t.time() - t_prev > 5000:
        cv2.imwrite(f'balls/frame_{num_of_frames:03d}_1.jpg', frame)
        num_of_frames += 1
        print(f"img dfa")
        t_prev = t.time()