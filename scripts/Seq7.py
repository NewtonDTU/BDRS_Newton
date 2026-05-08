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
    mainServo = MainServo()
    gripper = Gripper()
    hand = Hand()
    hand.close(200)
    mainServo.goto_up()
    t.sleep(1)
    gripper.open(600)
    utils.drive_distance(2.37, 0.3)
    utils.driveTurn(np.pi, 0.5)
    utils.drive_until_line(0.4)
    utils.follow_line_for_distance(0.2, 0.2, "right")
    utils.follow_line_for_distance(1.1, 0.2, "left")
    utils.driveTurn(np.pi, 0.8)
    utils.follow_line_until_intersection(0.2, "center")
    utils.follow_line_for_distance(0.2, 0.6, "center")

    utils.stop_robot()

  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()
    