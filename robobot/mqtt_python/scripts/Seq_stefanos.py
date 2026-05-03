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
from ball_alignment2 import run_ball_alignment
import cv2

def run():  
  aruco_detected = False
  hand = Hand()
  mainServo = MainServo()
  gripper = Gripper()
  try:
    hand.close(200)
    mainServo.goto_middle()
    gripper.open(600)
    t.sleep(5)

    run_ball_alignment("RED_BALL") #goes to 12 and 13
    utils.drive_distance(0.12, 0.2)

    mainServo.goto_down(30)
    gripper.close(300)
    t.sleep(1)

    



  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()