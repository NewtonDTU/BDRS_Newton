# START to ROUNDABOUT


from datetime import *
from setproctitle import setproctitle
from uservice import service
from scripts import utils
from servo_control import Gripper, Hand, MainServo
import time as t

def run():
    try:
        mainServo = MainServo()
        hand = Hand()
        gripper = Gripper()
        hand.open(100)
        gripper.close_ball(200)
        mainServo.goto_up()
        t.sleep(4)
        utils.follow_line_for_distance(distance=1.5, velocity=0.40, mode="left")
        utils.follow_until_line_ends(velocity=0.5, mode="center")
        utils.stop_robot()

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")

if __name__ == "__main__":
    run()