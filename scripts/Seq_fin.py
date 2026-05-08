#!/usr/bin/env python3

import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
import numpy as np

from scripts import utils

from servo_control import Gripper, Servo, Hand, MainServo


def run():
    try:
        if service.connected:
            mainServo = MainServo()
            gripper = Gripper()
            hand = Hand()
            hand.open(100)

            mainServo.goto_up()
            gripper.close_ball(200)

            utils.drive_distance(0.1, 0.3)
            utils.drive_until_line(0.3)
            utils.driveTurn(np.pi/2, 0.7)
            utils.follow_line_for_distance(2.6, 0.5, "right")
            utils.follow_until_line_ends(0.1, "center")
            

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")

if __name__ == "__main__":
    run()