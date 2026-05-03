import os
import csv
from datetime import datetime

#!/usr/bin/env python3
import time as t
import cv2 as cv

from holedet import holedet
from uservice import service


STREAM_URL = "http://localhost:7123/stream.mjpg"

# smaller = higher in image
TARGET_Y_RATIO = 0.70
TARGET_X_RATIO = 0.45 #Higer = more to the left, smaller = more to the right

X_THRESHOLD = 3
Y_THRESHOLD = 12

MAX_TURN = 1.00
MIN_TURN = 0.07

MAX_SPEED = 0.20
MIN_SPEED = 0.05

LOST_TARGET_STOP = True
PRINT_EVERY = 0.05

#data start
DATA_DIR = "data"


def make_log_file(prefix="hole_alignment"):
    os.makedirs(DATA_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(DATA_DIR, f"{prefix}_{stamp}.csv")


def init_log_file(log_path):
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time",
            "detected",
            "hole_x",
            "hole_y",
            "target_x",
            "target_y",
            "err_x",
            "err_y",
            "turn_cmd",
            "speed_cmd",
            "x_aligned",
            "y_aligned",
            "phase"
        ])


def append_log_row(log_path, row):
    with open(log_path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)


#data end
def clamp(val, low, high):
    return max(low, min(high, val))


def stop_robot():
    service.send("robobot/cmd/ti", "rc 0 0")


def get_target_y(frame_height):
    return frame_height * TARGET_Y_RATIO


def rotation_alignment(target_x, frame_width, threshold=X_THRESHOLD,
                       max_turn=MAX_TURN, min_turn=MIN_TURN):
    center_x = frame_width * TARGET_X_RATIO
    error_x = center_x - target_x

    if abs(error_x) <= threshold:
        return True, error_x, 0.0

    norm = abs(error_x) / (frame_width / 2.0)
    norm = clamp(norm, 0.0, 1.0)

    turn_mag = min_turn + (max_turn - min_turn) * norm
    turn_cmd = abs(turn_mag) if error_x > 0 else -abs(turn_mag)

    return False, error_x, turn_cmd


def distance_alignment(target_y_pos, frame_height, threshold=Y_THRESHOLD,
                       max_speed=MAX_SPEED, min_speed=MIN_SPEED):
    target_y = get_target_y(frame_height)
    error_y = target_y - target_y_pos

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


def run_hole_alignment():
    #data start
    log_path = make_log_file("hole_alignment")
    init_log_file(log_path)
    print(f"% Logging hole alignment data to {log_path}")
    #data end
    last_debug_print = 0.0
    print("% Starting hole alignment")
    print(f"% Stream: {STREAM_URL}")
    print(f"% Target Y ratio: {TARGET_Y_RATIO:.2f}")

    cap = open_stream()
    if cap is None:
        print("% Failed to open camera stream")
        return False

    fail_count = 0
    last_print = 0.0
    last_aligned_print = 0.0
    phase = "rotate"

    try:
        while not service.stop:
            ok, frame = cap.read()

            if not ok or frame is None:
                fail_count += 1
                if LOST_TARGET_STOP:
                    stop_robot()
                print(f"% Failed to read frame ({fail_count})")
                if fail_count >= 10:
                    print("% Too many frame failures, stopping")
                    return False
                t.sleep(0.05)
                continue

            fail_count = 0
            result = holedet(frame)

            if result is None:
                if LOST_TARGET_STOP:
                    stop_robot()

                now = t.time()
                if now - last_print > PRINT_EVERY:
                    print(f"Hole not detected | phase={phase}")
                    last_print = now

                t.sleep(0.02)
                continue

            best, mask, hole_coords = result
            score, (x, y, w, h), (cx, cy), area, aspect, fill_ratio = best

            hole_x = hole_coords[0]
            hole_y = hole_coords[1]
            frame_width = frame.shape[1]
            frame_height = frame.shape[0]

            x_aligned, error_x, turn_cmd = rotation_alignment(hole_x, frame_width)
            y_aligned, error_y, speed_cmd, target_y = distance_alignment(hole_y, frame_height)

            now = t.time()
            if now - last_debug_print >= 0.4:
                print(
                    f"hole_x={hole_x:.1f}, hole_y={hole_y:.1f}, "
                    f"err_x={error_x:.1f}, err_y={error_y:.1f}, "
                    f"x_ok={x_aligned}, y_ok={y_aligned}, "
                    f"turn={turn_cmd:.3f}, speed={speed_cmd:.3f}, phase={phase}"
                )
                last_debug_print = now

            if x_aligned and y_aligned:
                stop_robot()
                now = t.time()
                if now - last_aligned_print > 0.3:
                    print(
                        f"HOLE ALIGNED | "
                        f"x={hole_x:.1f}, y={hole_y:.1f} | "
                        f"err_x={error_x:.1f}, err_y={error_y:.1f}"
                    )
                    last_aligned_print = now
                return True

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
                    f"Hole x={hole_x:.1f}, y={hole_y:.1f} | "
                    f"err_x={error_x:.1f}, err_y={error_y:.1f} | "
                    f"turn={turn_cmd:.3f}, speed={speed_cmd:.3f} | "
                    f"x_ok={x_aligned}, y_ok={y_aligned}, phase={phase}"
                )
                last_print = now

            t.sleep(0.02)

    finally:
        stop_robot()
        cap.release()
        print("% Hole alignment terminated")