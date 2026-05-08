#!/usr/bin/env python3

import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
from scripts.ball_alignment2 import run_ball_alignment, stop_robot
from servo_control import MainServo, Gripper
import scripts.utils as utils
from scripts.findAruco import find_aruco_markers

if __name__ == "__main__":
    setproctitle("sequence-3")

    service.setup("localhost")

    if service.connected:
        # Test the detection for RED and BLUE ball and grab them
        Gripper().open()
        MainServo(speed=30).goto_middle()
        t.sleep(0.5)

        run_ball_alignment("RED_BALL")      # Change between RED - BLUE BALL
        t.sleep(0.5)

        utils.drive_distance(0.12, speed=0.2)
        t.sleep(0.5)
        MainServo(speed=30).goto_down()
        Gripper().close()
        t.sleep(0.5)

        utils.drive_distance(0.12, speed=0.2)
        utils.rotate(1.9, 0.5)

        stop_robot()

    service.terminate()