import cv2
import numpy as np

DEFAULT_MIN_AREA = 500
DEFAULT_MIN_CIRC = 0.20

"""
Detects a colored ball in a frame using HSV thresholding and contour filtering.

input:
 - frame: BGR image
 - ball_color: 'ORANGE_BALL', 'BLUE_BALL', or 'RED_BALL'

returns:
 - best: (score, (x, y, w, h), (cx, cy), area, circ)
 - mask: binary image
 - ball_coords: np.array([cx, cy])

If no ball is found, returns None.
"""

def circularity(cnt) -> float:
    area = cv2.contourArea(cnt)
    peri = cv2.arcLength(cnt, True)
    if peri <= 1e-6:
        return 0.0
    return float(4.0 * np.pi * area / (peri * peri))


def balldet(frame, ball_color):
    ball_coords = np.array([0.0, 0.0])
    lower = None
    upper = None
    best = None

    # local defaults for this run
    min_area = DEFAULT_MIN_AREA
    min_circ = DEFAULT_MIN_CIRC

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if ball_color == "ORANGE_BALL":
        lower = np.array([0, 178, 145], dtype=np.uint8)
        upper = np.array([25, 255, 255], dtype=np.uint8)
        min_circ = 0.20

    elif ball_color == "BLUE_BALL":
        lower = np.array([80, 50, 97], dtype=np.uint8)
        upper = np.array([126, 255, 255], dtype=np.uint8)

    elif ball_color == "RED_BALL":
        # TODO: add real red range
        return None

    else:
        return None

    mask = cv2.inRange(hsv_frame, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

    if best is None:
        return None

    ball_coords = np.array([best[2][0], best[2][1]])
    return best, mask, ball_coords