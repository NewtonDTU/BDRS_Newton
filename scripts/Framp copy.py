#!/usr/bin/env python3

import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
from ball_alignment import run_ball_alignment, stop_robot
from servo_control import MainServo, Gripper
from hole_alignment import run_hole_alignment
import utils



if __name__ == "__main__":
    setproctitle("sequence-3")

    service.setup("localhost")

    if service.connected:
        edge.setup()
        Gripper().open()
        MainServo(speed=30).goto_up()

        utils.follow_line_until_tilt_back_normal(
            seconds=25,
            velocity=0.35,
            mode="right",
            tilt_up_limit=0.12,
            tilt_down_limit=0.10,
            up_hold_time=0.15,
            down_hold_time=0.03
        )
        
        utils.rotate(2, 1)
        
        t.sleep(1)
       
        MainServo(speed=30).goto_middle()

        run_ball_alignment("ORANGE_BALL")
        t.sleep(0.5)
        utils.drive_distance(0.12, speed=0.2)
        t.sleep(0.5)
        MainServo(speed=30).goto_down()
        t.sleep(0.5)
        Gripper().close()
        t.sleep(0.5)
        #MainServo(speed=30).goto_middle()
        print("first rotate")

        utils.drive_distance(0.12, speed=0.1)
        utils.rotate(2.35, 1)
        print("find line")
        utils.drive_until_line(speed=0.22)
        print("found line")
        t.sleep(0.5)
        print("deeper")
        utils.drive_distance(0.09, speed=0.1)
        t.sleep(0.5)
        print("second rotate")
        utils.rotate(1.9, 0.5)
        t.sleep(0.5)
        print("follow line for distance")
        utils.follow_line_for_distance(distance=0.35, velocity=0.15, mode="right")
        print("follow and turn")
        t.sleep(0.5)
        utils.follow_and_turn(0.10, 0.8)
        
        utils.drive_distance(0.30, speed=-0.15)
        t.sleep(0.5)
        run_hole_alignment()
        #49 cm
        utils.drive_distance(0.31, 0.15)
        #MainServo(speed=30).goto_down()
        t.sleep(0.5)
        MainServo(speed=30).goto_middle()
        Gripper().open()



        
        stop_robot()

        

    service.terminate()