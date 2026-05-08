#!/usr/bin/env python3

import time as t
from pathlib import Path

import cv2
import numpy as np
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from scam import cam
from uservice import service
from scripts import utils


def circularity(cnt):
    area = cv2.contourArea(cnt)
    peri = cv2.arcLength(cnt, True)
    if peri <= 1e-6:
        return 0.0
    return float(4.0 * np.pi * area / (peri * peri))


def detect_orange_ball(frame_bgr, params):
    hmin, hmax = params["hmin"], params["hmax"]
    smin = params["smin"]
    vmin = params["vmin"]
    min_area = params["min_area"]
    min_circ = params["min_circ"]
    morph_iter = params["morph_iter"]

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    hh0, hh1 = min(hmin, hmax), max(hmin, hmax)
    lower = np.array([hh0, smin, vmin], dtype=np.uint8)
    upper = np.array([hh1, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)

    if morph_iter > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=morph_iter)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=morph_iter)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        circ = circularity(cnt)
        if circ < min_circ:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue

        cx = x + w / 2.0
        cy = y + h / 2.0
        score = float(area) * float(circ)

        if best is None or score > best[0]:
            best = (score, (x, y, w, h), (cx, cy), area, circ)

    vis = frame_bgr.copy()
    det = None

    if best is not None:
        _, (x, y, w, h), (cx, cy), area, circ = best
        det = {
            "bbox": (x, y, w, h),
            "center": (cx, cy),
            "area": area,
            "circ": circ,
        }

        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(vis, (int(cx), int(cy)), 4, (255, 0, 0), -1)
        cv2.putText(
            vis,
            f"area={int(area)} circ={circ:.2f}",
            (x, max(0, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    return vis, mask, det


'''def follow_line_until_tilt_back_normal(
    seconds=25,
    velocity=0.20,
    mode="center",
    ref_position=0.0,
    tilt_up_limit=0.12,
    tilt_down_limit=0.10,
    up_hold_time=0.15,
    down_hold_time=0.03,
):
    edge.lineControl(velocity, mode=mode, refPosition=ref_position)

    start = t.time()
    baseline_tilt = pose.pose[3]

    on_ramp = False
    tilt_up_start = None
    tilt_down_start = None

    while not service.stop and (t.time() - start < seconds):
        tilt_delta = abs(pose.pose[3] - baseline_tilt)

        if not on_ramp:
            if tilt_delta > tilt_up_limit:
                if tilt_up_start is None:
                    tilt_up_start = t.time()
                elif (t.time() - tilt_up_start) >= up_hold_time:
                    print(True)
                    on_ramp = True
            else:
                tilt_up_start = None
        else:
            if tilt_delta < tilt_down_limit:
                if tilt_down_start is None:
                    tilt_down_start = t.time()
                elif (t.time() - tilt_down_start) >= down_hold_time:
                    break
            else:
                tilt_down_start = None

        t.sleep(0.01)

    stop_robot()'''


def rotate_ccw_67_5():
    target_angle = 1.45

    edge.lineControl(0, mode="center")
    t.sleep(0.1)
    pose.tripBreset()

    while not service.stop:
        service.send("robobot/cmd/ti", "rc 0.0 0.5")

        if pose.tripBh >= target_angle or pose.tripBtimePassed() > 5:
            break

        t.sleep(0.02)

    utils.stop_robot()
    t.sleep(0.2)


def align_to_golf_ball(timeout=10.0, turn_k=0.001, max_turn=0.08, center_tol_px=25):
    edge.lineControl(0, mode="center")
    t.sleep(0.2)

    params = {
        "hmin": 0,
        "hmax": 15,
        "smin": 100,
        "vmin": 185,
        "min_area": 80,
        "min_circ": 0.45,
        "morph_iter": 1,
    }

    debug_dir = Path(__file__).resolve().parent / "cam_debug"
    debug_dir.mkdir(exist_ok=True)

    start = t.time()
    centered_count = 0
    frame_idx = 0
    last_seen_dir = 0   # -1 = left, +1 = right

    while not service.stop and (t.time() - start < timeout):
        ok, frame, frame_time = cam.getImage()

        if not ok or frame is None:
            print("% CAM: no frame")
            service.send("robobot/cmd/ti", "rc 0 0")
            t.sleep(0.05)
            continue

        frame_idx += 1
        vis, mask, det = detect_orange_ball(frame, params)

        h, w = frame.shape[:2]
        frame_center_x = w / 2.0

        cv2.line(vis, (int(frame_center_x), 0), (int(frame_center_x), h - 1), (255, 255, 0), 2)

        if det is None:
            print(f"% CAM: frame {frame_idx}, no ball detected")
            centered_count = 0

            if last_seen_dir != 0:
                search_turn = 0.04 * last_seen_dir
                print(f"% CAM: searching rc 0.0 {search_turn:.3f}")
                service.send("robobot/cmd/ti", f"rc 0.0 {search_turn:.3f}")
            else:
                service.send("robobot/cmd/ti", "rc 0 0")

            if frame_idx % 10 == 0:
                cv2.imwrite(str(debug_dir / f"frame_{frame_idx:04d}_vis.jpg"), vis)
                cv2.imwrite(str(debug_dir / f"frame_{frame_idx:04d}_mask.jpg"), mask)

            t.sleep(0.05)
            continue

        ball_x = det["center"][0]
        error_x = ball_x - frame_center_x

        if error_x > 0:
            last_seen_dir = 1
        elif error_x < 0:
            last_seen_dir = -1

        print(
            f"% CAM: frame {frame_idx}, "
            f"ball_x={ball_x:.1f}, center_x={frame_center_x:.1f}, "
            f"error_x={error_x:.1f}, area={det['area']:.1f}, circ={det['circ']:.2f}"
        )

        cv2.putText(
            vis,
            f"err={error_x:.1f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )

        if frame_idx % 5 == 0:
            cv2.imwrite(str(debug_dir / f"frame_{frame_idx:04d}_vis.jpg"), vis)
            cv2.imwrite(str(debug_dir / f"frame_{frame_idx:04d}_mask.jpg"), mask)

        if abs(error_x) <= center_tol_px:
            centered_count += 1
            service.send("robobot/cmd/ti", "rc 0 0")
            print(f"% CAM: centered count {centered_count}")
            if centered_count >= 3:
                print("% CAM: BALL CENTERED")
                break
        else:
            centered_count = 0

            turnrate = -turn_k * error_x
            turnrate = max(-max_turn, min(max_turn, turnrate))

            print(f"% CAM: command rc 0.0 {turnrate:.3f}")
            service.send("robobot/cmd/ti", f"rc 0.0 {turnrate:.3f}")

        t.sleep(0.05)

    utils.stop_robot()

def run():
    try:
        utils.follow_line_until_tilt_back_normal(
            seconds=25,
            velocity=0.40,
            mode="right",
            ref_position=0.0,
            tilt_up_limit=0.12,
            tilt_down_limit=0.10,
            up_hold_time=0.15,
            down_hold_time=0.03,
        )

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")

'''rotate_ccw_67_5()

        align_to_golf_ball(
            timeout=10.0,
            turn_k=0.001,
            max_turn=0.08,
            center_tol_px=25,
        )'''


if __name__ == "__main__":
    run()