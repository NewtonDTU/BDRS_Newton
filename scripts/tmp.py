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
import numpy as np
from servo_control import Gripper, Hand, MainServo


def run():
    try:
        utils.stop_robot()
        gripper = Gripper()
        mainServo = MainServo()
        gripper.open(100)
        t.sleep(2)
        gripper.close_ball(200)
        t.sleep(2)
        gripper.open(100)
        t.sleep(2)
        gripper.close_luggage(200)
        # utils.drive_until_line(0.2)
        # utils.follow_line_for_distance(0.3, 0.3, "center")
        # utils.driveTurn(np.deg2rad(90), -0.7)
        # gripper.close(50)"""

        """utils.drive_distance(0.5, 0.3)
        utils.driveTurn(np.pi / 2, 0.5)
        utils.drive_distance(0.5, 0.3)"""

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti", "rc 0 0")
        service.send("robobot/cmd/ti", "rc 0 0")


if __name__ == "__main__":
    run()
