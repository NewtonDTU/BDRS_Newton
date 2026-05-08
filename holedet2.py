import cv2
import numpy as np


# =========================
# Tuned hole detector values
# =========================

LOWER = np.array([8, 23, 62], dtype=np.uint8)
UPPER = np.array([46, 145, 178], dtype=np.uint8)

# 5-corner platform polygon as percentage of image size.
# Replace these with the values printed from your test script.
#
# P1 = left upper corner
# P2 = top middle corner
# P3 = right upper corner
# P4 = bottom right corner
# P5 = bottom left corner
PLATFORM_POLY = [
    (0.24, 0.59),
    (0.47, 0.52),
    (0.71, 0.57),
    (0.67, 0.70),
    (0.21, 0.71),
]

MIN_AREA = 78
MAX_AREA = 1800

# 1.0 = circular, lower = more stretched oval
MIN_OVAL_RATIO = 0.10

BLUR_SIZE = 5
KERNEL_SIZE = 5


def make_platform_mask(h, w):
    mask = np.zeros((h, w), dtype=np.uint8)

    pts = []

    for px, py in PLATFORM_POLY:
        x = int(px * (w - 1))
        y = int(py * (h - 1))
        pts.append([x, y])

    pts = np.array([pts], dtype=np.int32)

    cv2.fillPoly(mask, pts, 255)

    return mask


def clean_mask(mask):
    kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)

    # Remove small noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Fill small gaps
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def get_oval_confidence(cnt):
    if len(cnt) < 5:
        return None, None

    ellipse = cv2.fitEllipse(cnt)
    (_, _), (axis1, axis2), _ = ellipse

    if axis1 <= 0 or axis2 <= 0:
        return None, None

    oval_ratio = min(axis1, axis2) / max(axis1, axis2)

    return oval_ratio, ellipse


def holedet(frame):
    """
    Robot-compatible return format:

    If no hole is detected:
        return None

    If hole is detected:
        return best, mask, hole_coords
    """

    if frame is None:
        return None

    h, w = frame.shape[:2]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (BLUR_SIZE, BLUR_SIZE), 0)

    color_mask = cv2.inRange(hsv, LOWER, UPPER)

    platform_mask = make_platform_mask(h, w)

    # Only keep color detections inside the platform polygon
    mask = cv2.bitwise_and(color_mask, platform_mask)

    mask = clean_mask(mask)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    best = None

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < MIN_AREA or area > MAX_AREA:
            continue

        oval_ratio, ellipse = get_oval_confidence(cnt)

        if oval_ratio is None:
            continue

        if oval_ratio < MIN_OVAL_RATIO:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)

        if bw <= 0 or bh <= 0:
            continue

        cx = x + bw / 2.0
        cy = y + bh / 2.0

        # Bigger and more oval-like blobs get better score
        score = area * oval_ratio

        candidate = (
            float(score),
            (int(x), int(y), int(bw), int(bh)),
            (float(cx), float(cy)),
            float(area),
            float(oval_ratio),
            ellipse,
        )

        if best is None or candidate[0] > best[0]:
            best = candidate

    if best is None:
        return None

    hole_coords = np.array([best[2][0], best[2][1]], dtype=np.float32)

    return best, mask, hole_coords