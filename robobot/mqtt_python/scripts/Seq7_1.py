#import sys
#import threading

# SECOND 90 SECS

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
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from scripts import utils
from servo_control import Gripper, Servo, Hand, MainServo
import cv2

def run():
  state = 0
  not_found_debouncer = 0
  found_debouncer = 0
  debounce_threshold = 5
  marker_pos_ee20 = None
  marker_pos_ee53 = None
  
  aruco_detected = False
  hand = Hand()
  mainServo = MainServo()
  gripper = Gripper()
  try:
    hand.close(200)
    mainServo.goto_middle()
    gripper.open(600)
    t.sleep(5)

    utils.drive_distance(2.47, 0.3)
    utils.driveTurn(np.pi/3, 0.5)
    utils.drive_distance(0.20, 0.2)
    utils.driveTurn(np.pi/12, 0.5)
    utils.drive_distance(0.05, 0.2)
    utils.driveTurn(np.pi/12, 0.5)
    hand.open(200)

    utils.detect_luggages()
    
    utils.driveTurn(np.pi/12, 0.5)
    utils.drive_distance(0.02, 0.2)
    utils.driveTurn(np.pi/20, 0.5)

    t.sleep(3)

    utils.drive_distance(0.8, 0.3)
    hand.close(200)
    t.sleep(2)
    utils.driveTurn(np.deg2rad(220), 0.7)

    detected_marker_id, _ = utils.detect_closest_luggage()
    utils.go_to_marker_id(detected_marker_id)

    utils.drive_distance(0.12, 0.1)
    mainServo.goto_down()
    t.sleep(2)
    gripper.close(600)
    t.sleep(2)
    mainServo.goto_semi_down()
    t.sleep(2)
    utils.drive_distance(0.2, -0.1)
    utils.driveTurn(np.pi*1.5, -0.3)

    utils.go_to_marker_id(17, 0.52)
    gripper.open(600)
    t.sleep(2)

  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()