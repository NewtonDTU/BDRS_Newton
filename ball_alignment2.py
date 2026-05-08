import time as t
import os
import csv
from datetime import datetime

import cv2 as cv

from balldet import balldet
from uservice import service


DEFAULT_BALL_COLOR = "ORANGE_BALL"
STREAM_URL = "http://localhost:7123/stream.mjpg"

# Data logging
DATA_DIR = "data"
SAVE_IMAGE_EVERY = 0.3   # seconds between saved debug image sets

# Alignment settings
TARGET_Y_RATIO = 0.8

# IMPORTANT:
# Your current robot script actually uses 44% of the image width here:
#     center_x = frame_width / 100 * 44
# So this keeps the same behavior, but makes it easier to log/draw.
TARGET_X_RATIO = 0.44

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


def get_target_x(frame_width):
    return frame_width * TARGET_X_RATIO


def get_target_y(frame_height):
    return frame_height * TARGET_Y_RATIO


def make_run_paths(prefix="ball_alignment"):
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
            "ball_color",
            "ball_x",
            "ball_y",
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
            "circ",
        ])


def append_log_row(log_path, row):
    with open(log_path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)


def draw_detection(frame, result):
    out = frame.copy()

    target_x = int(get_target_x(out.shape[1]))
    target_y = int(get_target_y(out.shape[0]))

    # Target lines
    cv.line(out, (target_x, 0), (target_x, out.shape[0]), (255, 255, 0), 2)
    cv.line(out, (0, target_y), (out.shape[1], target_y), (255, 255, 0), 2)

    # Small text background
    cv.rectangle(out, (0, 0), (out.shape[1], 40), (30, 30, 30), -1)
    cv.putText(
        out,
        "Ball detection",
        (10, 28),
        cv.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv.LINE_AA,
    )

    if result is None:
        cv.putText(
            out,
            "No ball detected",
            (10, out.shape[0] - 15),
            cv.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv.LINE_AA,
        )
        return out

    best, mask, ball_coords = result
    score, (x, y, w, h), (cx, cy), area, circ = best

    cv.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv.circle(out, (int(cx), int(cy)), 6, (0, 0, 255), -1)

    txt = f"({int(cx)}, {int(cy)}) A={area:.0f} C={circ:.2f}"
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

    return out


def masked_color_preview(frame, mask):
    return cv.bitwise_and(frame, frame, mask=mask)


def save_debug_images(image_dir, idx, frame, result):
    if result is None:
        mask_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        mask_gray[:] = 0

        mask = cv.cvtColor(mask_gray, cv.COLOR_GRAY2BGR)
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


def rotation_alignment(ball_x, frame_width, threshold=X_THRESHOLD,
                       max_turn=MAX_TURN, min_turn=MIN_TURN):
    center_x = get_target_x(frame_width)
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
    print(f"% Target X ratio: {TARGET_X_RATIO:.2f}")
    print(f"% Target Y ratio: {TARGET_Y_RATIO:.2f}")

    log_path, image_dir = make_run_paths("ball_alignment")
    init_log_file(log_path)

    print(f"% Logging ball alignment data to {log_path}")
    print(f"% Saving debug images to {image_dir}")

    cap = open_stream()
    if cap is None:
        print("% Failed to open camera stream")
        return False

    fail_count = 0
    last_print = 0.0
    last_aligned_print = 0.0
    last_image_save = 0.0
    image_idx = 0
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
                    return False

                t.sleep(0.05)
                continue

            fail_count = 0

            frame_width = frame.shape[1]
            frame_height = frame.shape[0]
            target_x = get_target_x(frame_width)
            target_y = get_target_y(frame_height)

            result = balldet(frame, ball_color)
            now = t.time()

            if result is None:
                if LOST_BALL_STOP:
                    stop_robot()

                append_log_row(log_path, [
                    now,
                    0,
                    ball_color,
                    "", "",
                    target_x,
                    target_y,
                    "", "",
                    "", "",
                    "", "",
                    phase,
                    "", "", "", "",
                    "", "",
                ])

                if now - last_image_save >= SAVE_IMAGE_EVERY:
                    save_debug_images(image_dir, image_idx, frame, None)
                    image_idx += 1
                    last_image_save = now

                if now - last_print > PRINT_EVERY:
                    print(f"Ball not detected | phase={phase}")
                    last_print = now

                t.sleep(0.02)
                continue

            best, mask, ball_coords = result
            score, (x, y, w, h), (cx, cy), area, circ = best

            ball_x = ball_coords[0]
            ball_y = ball_coords[1]

            x_aligned, error_x, turn_cmd = rotation_alignment(ball_x, frame_width)
            y_aligned, error_y, speed_cmd, target_y = distance_alignment(ball_y, frame_height)

            append_log_row(log_path, [
                now,
                1,
                ball_color,
                ball_x,
                ball_y,
                target_x,
                target_y,
                error_x,
                error_y,
                turn_cmd,
                speed_cmd,
                int(x_aligned),
                int(y_aligned),
                phase,
                x, y, w, h,
                area,
                circ,
            ])

            if now - last_image_save >= SAVE_IMAGE_EVERY:
                save_debug_images(image_dir, image_idx, frame, result)
                image_idx += 1
                last_image_save = now

            if x_aligned and y_aligned:
                stop_robot()

                if now - last_aligned_print > 0.3:
                    print(
                        f"BALL ALIGNED | "
                        f"x={ball_x:.1f}, y={ball_y:.1f} | "
                        f"err_x={error_x:.1f}, err_y={error_y:.1f}"
                    )
                    last_aligned_print = now

                # Save one final image set at the alignment moment.
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
