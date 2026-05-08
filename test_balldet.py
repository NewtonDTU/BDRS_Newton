#!/usr/bin/env python3
import time as t
import cv2 as cv

from scripts.balldet import balldet

BALL_COLOR = "BLUE_BALL"
STREAM_URL = "http://localhost:7123/stream.mjpg"


def main():
    print("% Starting direct camera ball test")
    print(f"% Opening stream: {STREAM_URL}")

    cap = cv.VideoCapture(STREAM_URL)

    if not cap.isOpened():
        print("% Failed to open camera stream")
        return

    fail_count = 0

    try:
        while True:
            ok, frame = cap.read()

            if not ok or frame is None:
                fail_count += 1
                print(f"% Failed to read frame ({fail_count})")
                if fail_count >= 10:
                    print("% Too many frame failures, stopping")
                    break
                t.sleep(0.05)
                continue

            fail_count = 0

            result = balldet(frame, BALL_COLOR)

            if result is not None:
                best, mask, ball_coords = result
                score, (x, y, w, h), (cx, cy), area, circ = best

                print(
                    f"Ball detected at x={ball_coords[0]:.1f}, y={ball_coords[1]:.1f} | "
                    f"bbox=({x},{y},{w},{h}) area={area:.1f} circ={circ:.2f} score={score:.1f}"
                )

            t.sleep(0.02)

    except KeyboardInterrupt:
        print("\n% Stopped by user")

    finally:
        cap.release()
        print("% Test terminated")


if __name__ == "__main__":
    main()