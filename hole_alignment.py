#!/usr/bin/env python3
import time as t
import os
import csv
from datetime import datetime

import cv2 as cv

from holedet2 import holedet #testing
#from holedet import holedet
from uservice import service


STREAM_URL = "http://localhost:7123/stream.mjpg"
DATA_DIR = "data"
# lower vlues is higher up, higher values is lower down
TARGET_Y_RATIO = 0.62
TARGET_X_RATIO = 0.447  # Higher = more to the left, smaller = more to the right

X_THRESHOLD = 6
Y_THRESHOLD = 6

MAX_TURN = 0.35
MIN_TURN = 0.05

MAX_SPEED = 0.20
MIN_SPEED = 0.05

LOST_TARGET_STOP = True
PRINT_EVERY = 0.05
SAVE_IMAGE_EVERY = 0.3


def clamp(val, low, high):
    return max(low, min(high, val))


def stop_robot():
    service.send("robobot/cmd/ti", "rc 0 0")


def get_target_y(frame_height):
    return frame_height * TARGET_Y_RATIO


def get_target_x(frame_width):
    return frame_width * TARGET_X_RATIO


def make_run_paths(prefix="hole_alignment"):
    os.makedirs(DATA_DIR, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    csv_path = os.path.join(DATA_DIR, f"{prefix}_{stamp}.csv")
    image_dir = os.path.join(DATA_DIR, f"{prefix}_{stamp}")

    os.makedirs(image_dir, exist_ok=True)

    return csv_path, image_dir


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
            "phase",
            "bbox_x",
            "bbox_y",
            "bbox_w",
            "bbox_h",
            "area",
            "oval_ratio",
            "score",
        ])


def append_log_row(log_path, row):
    with open(log_path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)


def draw_detection(frame, result):
    out = frame.copy()

    cv.rectangle(out, (0, 0), (out.shape[1], 40), (30, 30, 30), -1)
    cv.putText(
        out,
        "Hole detection",
        (10, 28),
        cv.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv.LINE_AA,
    )

    center_x = int(frame.shape[1] * TARGET_X_RATIO)
    target_y = int(frame.shape[0] * TARGET_Y_RATIO)

    cv.line(out, (center_x, 0), (center_x, out.shape[0]), (255, 255, 0), 2)
    cv.line(out, (0, target_y), (out.shape[1], target_y), (255, 255, 0), 2)

    if result is not None:
        best, mask, hole_coords = result
        score, (x, y, w, h), (cx, cy), area, oval_ratio, ellipse = best

        cv.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv.circle(out, (int(cx), int(cy)), 6, (0, 0, 255), -1)

        if ellipse is not None:
            cv.ellipse(out, ellipse, (255, 0, 255), 2)

        txt = (
            f"({int(cx)}, {int(cy)}) "
            f"A={area:.0f} oval={oval_ratio:.2f} score={score:.0f}"
        )

        cv.putText(
            out,
            txt,
            (10, out.shape[0] - 15),
            cv.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )

    else:
        cv.putText(
            out,
            "No hole detected",
            (10, out.shape[0] - 15),
            cv.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv.LINE_AA,
        )

    return out


def masked_color_preview(frame, mask):
    return cv.bitwise_and(frame, frame, mask=mask)


def save_debug_images(image_dir, idx, frame, result):
    if result is None:
        mask = cv.UMat(frame.shape[0], frame.shape[1], cv.CV_8UC1).get()
        mask[:] = 0
        mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)

        detected = draw_detection(frame, None)

        preview = frame.copy()
        preview[:] = 0

    else:
        best, mask_gray, _ = result

        mask = cv.cvtColor(mask_gray, cv.COLOR_GRAY2BGR)
        detected = draw_detection(frame, result)
        preview = masked_color_preview(frame, mask_gray)

    cv.imwrite(os.path.join(image_dir, f"frame_{idx:04d}.png"), frame)
    cv.imwrite(os.path.join(image_dir, f"mask_{idx:04d}.png"), mask)
    cv.imwrite(os.path.join(image_dir, f"detected_{idx:04d}.png"), detected)
    cv.imwrite(os.path.join(image_dir, f"preview_{idx:04d}.png"), preview)


def rotation_alignment(
    target_x,
    frame_width,
    threshold=X_THRESHOLD,
    max_turn=MAX_TURN,
    min_turn=MIN_TURN,
):
    center_x = get_target_x(frame_width)
    error_x = center_x - target_x

    if abs(error_x) <= threshold:
        return True, error_x, 0.0

    norm = abs(error_x) / (frame_width / 2.0)
    norm = clamp(norm, 0.0, 1.0)

    turn_mag = min_turn + (max_turn - min_turn) * norm

    # Flipped sign compared to before
    # If hole is left of target line, rotate toward it
    #turn_cmd = -abs(turn_mag) if error_x > 0 else abs(turn_mag)
    turn_cmd = abs(turn_mag) if error_x > 0 else -abs(turn_mag)
    return False, error_x, turn_cmd


def distance_alignment(
    target_y_pos,
    frame_height,
    threshold=Y_THRESHOLD,
    max_speed=MAX_SPEED,
    min_speed=MIN_SPEED,
):
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
    last_debug_print = 0.0
    last_image_save = 0.0
    image_idx = 0

    print("% Starting hole alignment")
    print(f"% Stream: {STREAM_URL}")
    print(f"% Target X ratio: {TARGET_X_RATIO:.2f}")
    print(f"% Target Y ratio: {TARGET_Y_RATIO:.2f}")

    log_path, image_dir = make_run_paths("hole_alignment")
    init_log_file(log_path)

    print(f"% Logging hole alignment data to {log_path}")
    print(f"% Saving debug images to {image_dir}")

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

            frame_width = frame.shape[1]
            frame_height = frame.shape[0]

            target_x = get_target_x(frame_width)
            target_y = get_target_y(frame_height)

            result = holedet(frame)

            if result is None:
                if LOST_TARGET_STOP:
                    stop_robot()

                append_log_row(log_path, [
                    t.time(),
                    0,
                    "",
                    "",
                    target_x,
                    target_y,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    phase,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                ])

                now = t.time()

                if now - last_image_save >= SAVE_IMAGE_EVERY:
                    save_debug_images(image_dir, image_idx, frame, None)
                    image_idx += 1
                    last_image_save = now

                if now - last_print > PRINT_EVERY:
                    print(f"Hole not detected | phase={phase}")
                    last_print = now

                t.sleep(0.02)
                continue

            best, mask, hole_coords = result
            score, (x, y, w, h), (cx, cy), area, oval_ratio, ellipse = best

            hole_x = hole_coords[0]
            hole_y = hole_coords[1]

            x_aligned, error_x, turn_cmd = rotation_alignment(hole_x, frame_width)
            y_aligned, error_y, speed_cmd, target_y = distance_alignment(hole_y, frame_height)

            append_log_row(log_path, [
                t.time(),
                1,
                hole_x,
                hole_y,
                target_x,
                target_y,
                error_x,
                error_y,
                turn_cmd,
                speed_cmd,
                int(x_aligned),
                int(y_aligned),
                phase,
                x,
                y,
                w,
                h,
                area,
                oval_ratio,
                score,
            ])

            now = t.time()

            if now - last_image_save >= SAVE_IMAGE_EVERY:
                save_debug_images(image_dir, image_idx, frame, result)
                image_idx += 1
                last_image_save = now

            if now - last_debug_print >= 0.4:
                print(
                    f"hole_x={hole_x:.1f}, hole_y={hole_y:.1f}, "
                    f"err_x={error_x:.1f}, err_y={error_y:.1f}, "
                    f"x_ok={x_aligned}, y_ok={y_aligned}, "
                    f"oval={oval_ratio:.2f}, score={score:.0f}, "
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
                        f"err_x={error_x:.1f}, err_y={error_y:.1f} | "
                        f"oval={oval_ratio:.2f}"
                    )

                    last_aligned_print = now

                save_debug_images(image_dir, image_idx, frame, result)

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

            if now - last_print > PRINT_EVERY:
                print(
                    f"Hole x={hole_x:.1f}, y={hole_y:.1f} | "
                    f"err_x={error_x:.1f}, err_y={error_y:.1f} | "
                    f"oval={oval_ratio:.2f}, score={score:.0f} | "
                    f"turn={turn_cmd:.3f}, speed={speed_cmd:.3f} | "
                    f"x_ok={x_aligned}, y_ok={y_aligned}, phase={phase}"
                )

                last_print = now

            t.sleep(0.02)

    finally:
        stop_robot()
        cap.release()
        print("% Hole alignment terminated")


if __name__ == "__main__":
    service.setup("localhost")

    if service.connected:
        run_hole_alignment()

    service.terminate()