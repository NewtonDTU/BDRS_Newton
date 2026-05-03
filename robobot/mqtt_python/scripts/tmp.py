#import sys
#import threading
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
import numpy as np
from servo_control import Gripper, Hand, MainServo

def run():
    try:
        #utils.stop_robot()
        gripper = Gripper()

        mainServo = MainServo()

        mainServo.goto_up()

        #mainServo.goto_semi_down()
        #t.sleep(2)
        #utils.drive_distance(0.2, 0.2)

        #utils.drive_distance(0.1, -0.2)
        #utils.driveTurn(np.pi, -0.5)
        #t.sleep(2)
        """for i in range(5):
            gripper.close(600)
            t.sleep(3)
            gripper.open(600)
            t.sleep(3)
"""
        #mainServo.goto_up()
        #utils.follow_line_until_angle(np.pi/4, 0.3, "center")
        #utils.follow_line_for_distance(2.15, 0.3, "center")


    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")

if __name__ == "__main__":
    run()