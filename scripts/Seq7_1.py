# import sys
# import threading

# SECOND 90 SECS

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
import cv2


def run():
    detected_marker_id = None
    hand = Hand()
    mainServo = MainServo()
    gripper = Gripper()
    try:
        hand.open(100)
        mainServo.goto_middle()
        gripper.open(200)

        utils.drive_distance(0.4, 0.3)
        utils.drive_until_line(-0.2)

        utils.drive_distance(2.20, 0.3)
        utils.driveTurn(np.pi / 3, 0.5)
        utils.drive_distance(0.20, 0.2)
        utils.driveTurn(np.pi / 12, 0.5)
        utils.drive_distance(0.08, 0.2)
        utils.driveTurn(np.pi / 12, 0.5)
        utils.drive_distance(0.15, -0.2)
        hand.close(200)

        utils.detect_luggages()

        utils.driveTurn(np.pi / 12, 0.5)
        utils.drive_distance(0.02, 0.2)
        utils.driveTurn(np.pi / 20, 0.5)

        t.sleep(2.5)

        utils.driveCircle(np.pi / 6, 0.6, 0.8)
        utils.drive_distance(0.35, 0.4)
        hand.open(200)
        utils.driveTurn(np.deg2rad(210), 1.2)

        detected_marker_id = utils.try_pickup(gripper, mainServo)
        if detected_marker_id is not None:
            print("Successfully picked up luggage")
            if detected_marker_id == 20:
                utils.driveTurn(np.pi / 2, 1.2)
                utils.drive_distance(0.4, 0.4)

                utils.go_to_marker_id(
                    17, 0.60, dir=1, angle_tolerance=np.deg2rad(5), turn_speed=0.3
                )
                utils.driveTurn(np.deg2rad(75), -0.5)
                utils.driveCircle(np.pi / 3, 0.3, 0.4)
                utils.driveTurn(np.pi / 3, 0.7)
                utils.go_to_marker_id(
                    11, 0.45, dir=1, angle_tolerance=np.deg2rad(5), turn_speed=0.3
                )
                gripper.open(600)
                utils.drive_distance(0.32, -0.3)
                utils.driveTurn(np.pi / 2, 0.9)
                utils.driveCircle(np.pi / 2, 0.3, -0.4)
                utils.driveTurn(np.pi / 2, 0.9)
                if utils.try_pickup(gripper, mainServo, 53):
                    utils.drive_distance(0.2, -0.3)
                    utils.driveTurn(np.pi / 2, 1.2)
                    utils.drive_distance(0.3, 0.4)
                    utils.go_to_marker_id(
                        17, 0.45, dir=1, angle_tolerance=np.deg2rad(5), turn_speed=0.3
                    )
                    gripper.open(600)
                    utils.drive_distance(0.2, -0.3)
                    utils.driveTurn(np.pi / 3, -1.2)
                    utils.drive_distance(1.15, 0.5)
                    utils.driveTurn(np.deg2rad(30), 1.2)
                    utils.drive_until_line(0.2)
                    utils.follow_line_for_distance(0.3, 0.3, "center")
                    utils.driveTurn(np.deg2rad(90), -0.7)

            if detected_marker_id == 53:
                utils.driveTurn(np.pi / 2, 1.2)
                utils.drive_distance(0.3, 0.4)

                utils.go_to_marker_id(
                    17, 0.45, dir=1, angle_tolerance=np.deg2rad(5), turn_speed=0.3
                )
                gripper.open(600)
                utils.drive_distance(0.2, -0.3)
                utils.driveTurn(np.deg2rad(160), 1.2)
                if utils.try_pickup(gripper, mainServo, 20):
                    utils.driveTurn(np.pi / 2, 1.2)
                    utils.go_to_marker_id(
                        17, 0.60, dir=1, angle_tolerance=np.deg2rad(5), turn_speed=0.3
                    )
                    utils.driveTurn(np.deg2rad(75), -0.75)
                    utils.driveCircle(np.pi / 3, 0.3, 0.4)
                    utils.driveTurn(np.pi / 3, 1.2)
                    utils.go_to_marker_id(
                        11, 0.45, dir=1, angle_tolerance=np.deg2rad(5), turn_speed=0.3
                    )
                    gripper.open(600)
                    utils.drive_distance(0.3, -0.3)
                    utils.driveTurn(np.deg2rad(120), -0.5)
                    utils.drive_until_line(0.2)
                    utils.driveTurn(np.deg2rad(30), 0.7)
                    utils.follow_line_for_distance(0.3, 0.2, "center")
                    utils.drive_distance(0.3, -0.2)
                    utils.follow_line_for_distance(0.2, 0.2, "center")
                    utils.driveTurn(np.deg2rad(90), -0.7)
        else:
            print("Failed to pick up luggage")
            gripper.open(200)
            utils.go_to_marker_id(
                17, 0.6, dir=1, angle_tolerance=np.deg2rad(5), turn_speed=0.3
            )
            utils.driveTurn(np.deg2rad(75), -0.75)
            utils.driveCircle(np.pi / 3, 0.3, 0.45)
            utils.driveTurn(np.deg2rad(30), -0.5)

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti", "rc 0 0")
        service.send("robobot/cmd/ti", "rc 0 0")


if __name__ == "__main__":
    run()
