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
from scripts.ball_alignment2 import run_ball_alignment

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
    
    utils.drive_distance(0.30, 0.2)
    utils.driveTurn((np.pi/2), 0.5)
    utils.drive_distance(0.20, 0.2)
    utils.driveTurn(np.pi/8, 0.5)

    # Find and place blue ball
    t.sleep(1)
    run_ball_alignment("BLUE_BALL") #goes to blue red
    utils.drive_distance(0.12, 0.2)
    mainServo.goto_down(30)   #goto_semi_down  goto_down (30)
    gripper.close(300)

    utils.go_to_marker_id(15, 0.52)
    gripper.open(600)
    mainServo.goto_up()

    #Move from blue ball placement to search for red ball and place on zone
    utils.drive_distance(0.1, -0.2)
    utils.driveTurn(np.pi/2, -0.5) # right turn
    mainServo.goto_down(30)
    utils.drive_distance(0.5, 0.2)
    utils.driveTurn(np.pi*0.9, -0.5)

    run_ball_alignment("RED_BALL") #goes to blue red
    utils.drive_distance(0.12, 0.2)
    mainServo.goto_semi_down()
    gripper.close(300)

    utils.go_to_marker_id(12, 0.52)
    gripper.open(600)
    mainServo.goto_up()
    utils.driveTurn((np.pi/2), 0.5)
    utils.drive_distance(0.5, 0.2)

    utils.stop_robot()


  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()