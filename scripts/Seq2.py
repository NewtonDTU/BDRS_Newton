# ROUNDABOUT

# import sys
# import threading
import time as t

# import select
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

def run():
    try:
        hand = Hand()
        hand.open(100)
        utils.drive_distance(0.25, 0.5)
        utils.driveTurn(2 * np.pi / 4, -0.7)
        utils.drive_distance(0.20, 0.15)
        utils.driveTurn(np.deg2rad(55), 0.7)
        utils.drive_distance(0.1, 0.1)
        utils.driveCircle(np.deg2rad(405), 0.4, 1.2)
        utils.driveTurn(np.deg2rad(45), -0.7)
        # _, mode = utils.drive_until_line(0.2, 0.5, True)
        utils.drive_distance(0.6, 0.2)
        utils.driveTurn(np.deg2rad(90), 0.7)
        utils.drive_until_line(0.2)
        utils.follow_line_for_distance(0.3, 0.2, "right")
        utils.follow_line_for_distance(0.3, 0.2, "center")
        utils.follow_line_until_angle(np.pi / 4, 0.5, "center")
        utils.stop_robot()

        # utils.driveCircle()

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti", "rc 0 0")
        service.send("robobot/cmd/ti", "rc 0 0")


if __name__ == "__main__":
    run()
