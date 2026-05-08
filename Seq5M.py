#!/usr/bin/env python3
import time as t
import numpy as np
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
from servo_control import MainServo, Gripper
from hole_alignment import run_hole_alignment
import utils
from ball_alignment import run_ball_alignment


# =========================
# Tune these values
# =========================

BALL_COLOR = "ORANGE_BALL"
LINE_MODE = "right"

# First green line/ramp part
RAMP_LINE_SPEED = 0.35
TOP_LINE_DISTANCE = 0.45

# Ball pickup
BALL_EXTRA_FORWARD = 0.10
BALL_PICKUP_SPEED = 0.15
BACKUP_AFTER_PICKUP = 0.08

# Rotate back after ball alignment
RETURN_TURN_SPEED = 0.50

# If rotate-back turns wrong way, change this from -1.0 to 1.0
RETURN_ROTATION_MULTIPLIER = -1.0

# Red path after ball pickup
RED_PATH_DISTANCE = 1.00
RED_PATH_SPEED = 0.18

# Final hole delivery
HOLE_FORWARD_DISTANCE = 0.31
HOLE_FORWARD_SPEED = 0.15


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
    print("% Preparing for ball alignment")

    MainServo(speed=30).goto_middle()
    t.sleep(0.5)

    

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
        t.sleep(0.3)

    MainServo(speed=30).goto_down()
    t.sleep(0.5)

    Gripper().close()
    t.sleep(0.5)

    # Back away slightly before rotating back
    if BACKUP_AFTER_PICKUP > 0:
        utils.drive_distance(BACKUP_AFTER_PICKUP, speed=-0.12)
        t.sleep(0.3)

    return ball_angle


def main():
    setproctitle("simple-red-route")

    service.setup("localhost")

    try:
        if not service.connected:
            print("% Service not connected")
            return

        edge.setup()


        gripper = Gripper()
        servo = MainServo(speed=30)

        gripper.open()
        t.sleep(0.3)
        servo.goto_up()
        t.sleep(0.5)

        # =========================
        # 1. Follow line over ramp
        # =========================
        print("% Follow line over ramp")

        utils.follow_line_until_tilt_back_normal(
            seconds=25,
            velocity=RAMP_LINE_SPEED,
            mode=LINE_MODE,
            tilt_up_limit=0.12,
            tilt_down_limit=0.10,
            up_hold_time=0.15,
            down_hold_time=0.03,
        )

        safe_stop()
        t.sleep(0.5)

        # =========================
        # 2. Hardcoded turn toward ball
        # =========================
        print("% Hardcoded turn toward ball")

        utils.rotate(2.0, 1.0)   # tune angle and direction
        safe_stop()
        t.sleep(0.5)

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

        safe_stop()
        t.sleep(0.5)

        # =========================
        # 6. Hole alignment and release
        # =========================
        print("% Starting hole alignment")

        run_hole_alignment()

        safe_stop()
        t.sleep(0.5)

        utils.drive_distance(
            distance=HOLE_FORWARD_DISTANCE,
            speed=HOLE_FORWARD_SPEED,
        )

        t.sleep(0.5)

        servo.goto_middle()
        t.sleep(0.5)

        gripper.open()
        t.sleep(0.5)

        safe_stop()
        print("% Simple red route complete")

    finally:
        safe_stop()
        service.terminate()


if __name__ == "__main__":
    main()