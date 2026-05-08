#!/usr/bin/env python3

# SECOND RAMP

import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
import numpy as np
from servo_control import Gripper, Hand, MainServo

from scripts import utils

def run():
    try:
        hand = Hand()
        hand.open(200)

        mainServo = MainServo()
        mainServo.goto_up()
        t.sleep(4)
        utils.driveTurn(np.pi - np.pi/6, 0.5)
        utils.drive_until_line(speed=0.2)
        utils.follow_line_until_tilt_back_normal(
            seconds=25,
            velocity=0.3,
            mode="center",
            ref_position=0.0,
            tilt_up_limit=0.12,
            tilt_down_limit=0.10,
            up_hold_time=0.15,
            down_hold_time=0.03,
        )
        utils.stop_robot()
            

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")

if __name__ == "__main__":
    run()