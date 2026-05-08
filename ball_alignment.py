#!/usr/bin/env python3
import time as t
import cv2 as cv

from balldet import balldet
from uservice import service


DEFAULT_BALL_COLOR = "ORANGE_BALL"
STREAM_URL = "http://localhost:7123/stream.mjpg"

TARGET_Y_RATIO = 0.8
Target_X_RATIO = 0.46  # Higher = more to the left, smaller = more to the right
X_THRESHOLD = 6
Y_THRESHOLD = 9

MAX_TURN = 0.30
MIN_TURN = 0.05

MAX_SPEED = 0.20
MIN_SPEED = 0.05

LOST_BALL_STOP = True
PRINT_EVERY = 0.05


def clamp(val, low, high):
    return max(low, min(high, val))


def stop_robot():
    service.send("robobot/cmd/ti", "rc 0 0")


def get_target_y(frame_height):
    return frame_height * TARGET_Y_RATIO


def rotation_alignment(ball_x, frame_width, threshold=X_THRESHOLD,
                       max_turn=MAX_TURN, min_turn=MIN_TURN):
    center_x = frame_width / 100 * 44
    error_x = center_x - ball_x

    if abs(error_x) <= threshold:
        return True, error_x, 0.0

    norm = abs(error_x) / (frame_width / 2.0)
    norm = clamp(norm, 0.0, 1.0)

    turn_mag = min_turn + (max_turn - min_turn) * norm
    turn_cmd = abs(turn_mag) if error_x > 0 else -abs(turn_mag)

    return False, error_x, turn_cmd


def distance_alignment(ball_y, frame_height, threshold=Y_THRESHOLD,
                       max_speed=MAX_SPEED, min_speed=MIN_SPEED):
    target_y = get_target_y(frame_height)
    error_y = target_y - ball_y

    if abs(error_y) <= threshold:
        return True, error_y, 0.0, target_y

    norm = abs(error_y) / frame_height
    norm = clamp(norm, 0.0, 1.0)

    speed_mag = min_speed + (max_speed - min_speed) * norm
    speed_cmd = abs(speed_mag) if error_y > 0 else -abs(speed_mag)

    return False, error_y, speed_cmd, target_y


def open_stream():
    cap = cv.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        return None
    return cap


def run_ball_alignment(ball_color=DEFAULT_BALL_COLOR):
    print("% Starting ball alignment + movement")
    print(f"% Ball color: {ball_color}")
    print(f"% Stream: {STREAM_URL}")
    print(f"% Target Y ratio: {TARGET_Y_RATIO:.2f}")

    cap = open_stream()
    if cap is None:
        print("% Failed to open camera stream")
        return

    fail_count = 0
    last_print = 0.0
    last_aligned_print = 0.0
    phase = "rotate"

    try:
        while not service.stop:
            ok, frame = cap.read()

            if not ok or frame is None:
                fail_count += 1
                if LOST_BALL_STOP:
                    stop_robot()
                print(f"% Failed to read frame ({fail_count})")
                if fail_count >= 10:
                    print("% Too many frame failures, stopping")
                    break
                t.sleep(0.05)
                continue

            fail_count = 0
            result = balldet(frame, ball_color)

            if result is None:
                if LOST_BALL_STOP:
                    stop_robot()

                now = t.time()
                if now - last_print > PRINT_EVERY:
                    print(f"Ball not detected | phase={phase}")
                    last_print = now

                t.sleep(0.02)
                continue

            best, mask, ball_coords = result
            score, (x, y, w, h), (cx, cy), area, circ = best

            ball_x = ball_coords[0]
            ball_y = ball_coords[1]
            frame_width = frame.shape[1]
            frame_height = frame.shape[0]

            x_aligned, error_x, turn_cmd = rotation_alignment(ball_x, frame_width)
            y_aligned, error_y, speed_cmd, target_y = distance_alignment(ball_y, frame_height)

            if x_aligned and y_aligned:
                stop_robot()
                now = t.time()
                if now - last_aligned_print > 0.3:
                    print(
                        f"BALL ALIGNED | "
                        f"x={ball_x:.1f}, y={ball_y:.1f} | "
                        f"err_x={error_x:.1f}, err_y={error_y:.1f}"
                    )
                    last_aligned_print = now
                break

            if phase == "rotate":
                if not x_aligned:
                    service.send("robobot/cmd/ti", f"rc 0.0 {turn_cmd:.3f}")
                else:
                    phase = "move"

            elif phase == "move":
                if not y_aligned:
                    service.send("robobot/cmd/ti", f"rc {speed_cmd:.3f} 0.0")
                else:
                    phase = "rotate"

            now = t.time()
            if now - last_print > PRINT_EVERY:
                print(
                    f"Ball x={ball_x:.1f}, y={ball_y:.1f} | "
                    f"err_x={error_x:.1f}, err_y={error_y:.1f} | "
                    f"turn={turn_cmd:.3f}, speed={speed_cmd:.3f} | "
                    f"x_ok={x_aligned}, y_ok={y_aligned}, phase={phase}"
                )
                last_print = now

            t.sleep(0.02)

    finally:
        stop_robot()
        cap.release()
        print("% Ball alignment terminated")


if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        run_ball_alignment("ORANGE_BALL")
    service.terminate()