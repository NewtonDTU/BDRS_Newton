# ROUNDABOUT

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



def run():
    try:
        utils.drive_distance(0.3, 0.5)
        utils.driveTurn(2*np.pi/4, -0.5)
        utils.drive_distance(0.23, 0.15)
        utils.driveTurn(np.pi/6 + 0.3, 0.5)
        utils.drive_distance(0.1, 0.1)
        utils.driveCircle(2*np.pi + np.pi/2 + 0.05, 0.2, 0.6)
        utils .driveTurn(np.pi/2 - 0.1, -0.5)
        _, mode = utils.drive_until_line(0.2, 0.5, True)
        utils.follow_line_for_distance(0.2, 0.2, mode)
        utils.follow_line_until_angle(np.pi/4, 0.3, "center")
        utils.stop_robot()
        
        # utils.driveCircle()

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()