# 90 SECS

import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
from uservice import service
from scripts import utils
from servo_control import Gripper, Servo, Hand, MainServo

def run():
  try:
    hand = Hand()
    hand.open(100)
    utils.driveTurn(np.pi/4, -0.70)
    utils.drive_distance(0.70, 0.50)
    utils.driveTurn(np.pi/4, -0.70)
    utils.drive_distance(0.85, 0.50)
    utils.driveTurn(np.pi, -0.7)
    utils.drive_distance(0.85, 0.50)
    utils.drive_until_line(0.3)
    utils.follow_line_for_distance(2.35, 0.4, "right")

  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()