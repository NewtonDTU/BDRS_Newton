#!/usr/bin/env python3

# FIRST RAMP and BALL ALIGNMENT


#!/usr/bin/env python3
import time as t
import numpy as np
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
from servo_control import MainServo, Gripper, Hand
from hole_alignment import run_hole_alignment
from scripts import utils
from ball_alignment2 import run_ball_alignment

# =========================
# Tune these values
# =========================

BALL_COLOR = "ORANGE_BALL"
LINE_MODE = "right"

# First green line/ramp part
RAMP_LINE_SPEED = 0.35
TOP_LINE_DISTANCE = 0.45

# Ball pickup
BALL_EXTRA_FORWARD = 0.05
BALL_PICKUP_SPEED = 0.15
BACKUP_AFTER_PICKUP = 0.16

# Rotate back after ball alignment
RETURN_TURN_SPEED = 0.50

# If rotate-back turns wrong way, change this from -1.0 to 1.0
RETURN_ROTATION_MULTIPLIER = -1.5

# Red path after ball pickup
RED_PATH_DISTANCE = 0.50
RED_PATH_SPEED = 0.25

# Final hole delivery
HOLE_FORWARD_DISTANCE = 0.33

HOLE_FORWARD_SPEED = 0.2


def safe_stop(repeats=2):
    for _ in range(repeats):
        service.send("robobot/cmd/ti", "rc 0 0")
        t.sleep(0.05)


def rotate_back_from_ball_angle(ball_angle_rad, turn_speed=RETURN_TURN_SPEED):
    angle = abs(ball_angle_rad)

    if angle < 0.05:
        print("% Ball angle very small, skipping rotate back")
        return

    measured_sign = 1.0 if ball_angle_rad > 0 else -1.0
    return_sign = measured_sign * RETURN_ROTATION_MULTIPLIER
    signed_turn_speed = return_sign * abs(turn_speed)

    print(
        f"% Rotating back from ball angle: "
        f"{ball_angle_rad:.3f} rad ({np.rad2deg(ball_angle_rad):.1f} deg)"
    )
    print(f"% Return turn speed: {signed_turn_speed:.3f}")

    utils.rotate(angle, signed_turn_speed)
    safe_stop()
    t.sleep(0.3)


def align_and_pick_ball():
    gripper = Gripper()
    servo = MainServo(speed=30)
    print("% Preparing for ball alignment")

    t.sleep(1)

    run_ball_alignment(BALL_COLOR)

    safe_stop()
    t.sleep(0.2)

    ball_angle = pose.tripBh

    print(
        f"% Measured ball alignment angle: "
        f"{ball_angle:.3f} rad ({np.rad2deg(ball_angle):.1f} deg)"
    )

    # Small extra forward push to reach the ball
    if BALL_EXTRA_FORWARD > 0:
        utils.drive_distance(BALL_EXTRA_FORWARD, speed=BALL_PICKUP_SPEED)
        t.sleep(0.5)

    servo.goto_down()
    t.sleep(0.5)

    gripper.close_ball()

    t.sleep(0.5)

    servo.goto_middle()
    t.sleep(0.3)

    # Back awa
    # y slightly before rotating back
    if BACKUP_AFTER_PICKUP > 0:
        utils.drive_distance(BACKUP_AFTER_PICKUP, speed=0.12)

    return ball_angle


def run():
    try:
        print("Seq3_1_1: BALL ALIGNMENT")
        gripper = Gripper()
        servo = MainServo(speed=30)
        hand = Hand()
        #'''
        gripper.open()
        hand.open(200)
        servo.goto_up()

        # =========================
        # 1. Follow line over ramp
        # =========================
        print("% Follow line over ramp")

        '''utils.follow_line_until_tilt_back_normal(
            seconds=25,
            velocity=RAMP_LINE_SPEED,
            mode=LINE_MODE,
            tilt_up_limit=0.12,
            tilt_down_limit=0.07,
            up_hold_time=0.15,
            down_hold_time=0.03
        )'''
        utils.follow_line_for_distance(4.23, 0.6, "right")
        servo.goto_middle()
        safe_stop()
        t.sleep(0.5)

        # =========================
        # 2. Hardcoded turn toward ball
        # =========================
        print("% Hardcoded turn toward ball")

        utils.rotate(2.3, 1.0)  # tune angle and direction
        safe_stop()

        # =========================
        # 3. Align to ball and pick it
        # =========================
        print("% Align and pick ball")

        ball_angle = align_and_pick_ball()

        # =========================
        # 4. Rotate back to original heading
        # =========================
        rotate_back_from_ball_angle(ball_angle)

        # =========================
        # 5. Drive red path directly
        # =========================
        print("% Driving red path directly")

        utils.drive_distance(
            distance=RED_PATH_DISTANCE,
            speed=RED_PATH_SPEED,
        )
        #'''
        utils.rotate(0.65, -1.5)  # tune angle and direction
        servo.goto_semi_down()
        safe_stop()

        # =========================
        # 6. Hole alignment and release
        # =========================
        print("% Starting hole alignment")

        run_hole_alignment()

        safe_stop()

        print("hole found")
        utils.drive_distance(
            distance=HOLE_FORWARD_DISTANCE,
            speed=HOLE_FORWARD_SPEED,
        )
        print("drive forward")
        t.sleep(0.5)

        servo.goto_down()
        t.sleep(0.5)

        gripper.open()
        t.sleep(0.5)

        safe_stop()
        print("% Simple red route complete")

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti", "rc 0 0")
        service.send("robobot/cmd/ti", "rc 0 0")


if __name__ == "__main__":
    run()
