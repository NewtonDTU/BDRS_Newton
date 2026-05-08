import cv2
import numpy as np


# =========================
# Tuned hole detector values
# =========================

LOWER = np.array([8, 48, 71], dtype=np.uint8)
UPPER = np.array([40, 135, 181], dtype=np.uint8)

ROI_X_MIN = 0.20
ROI_X_MAX = 0.68
ROI_Y_MIN = 0.50
ROI_Y_MAX = 0.70

MIN_AREA = 78
MAX_AREA = 1800

MIN_OVAL_RATIO = 0.10

BLUR_SIZE = 5
KERNEL_SIZE = 5
'''
LOWER = np.array([13, 38, 91], dtype=np.uint8)
UPPER = np.array([67, 133, 178], dtype=np.uint8)

ROI_X_MIN = 0.25
ROI_X_MAX = 0.68
ROI_Y_MIN = 0.50
ROI_Y_MAX = 0.72

MIN_AREA = 78
MAX_AREA = 1800

MIN_OVAL_RATIO = 0.10

BLUR_SIZE = 5
KERNEL_SIZE = 5
'''

def clean_mask(mask):
    kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)

    # Remove small noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Fill small gaps
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def shift_ellipse(ellipse, dx, dy):
    (cx, cy), axes, angle = ellipse
    return ((cx + dx, cy + dy), axes, angle)


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
    if frame is None:
        return None

    h, w = frame.shape[:2]

    x0 = int(ROI_X_MIN * w)
    x1 = int(ROI_X_MAX * w)
    y0 = int(ROI_Y_MIN * h)
    y1 = int(ROI_Y_MAX * h)

    if x1 <= x0 or y1 <= y0:
        return None

    roi = frame[y0:y1, x0:x1]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (BLUR_SIZE, BLUR_SIZE), 0)

    mask_roi = cv2.inRange(hsv, LOWER, UPPER)
    mask_roi = clean_mask(mask_roi)

    contours, _ = cv2.findContours(
        mask_roi,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
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

        full_x = x + x0
        full_y = y + y0
        full_cx = cx + x0
        full_cy = cy + y0
        full_ellipse = shift_ellipse(ellipse, x0, y0)

        # Score prefers larger and more oval/circular blobs
        score = area * oval_ratio

        candidate = (
            float(score),
            (int(full_x), int(full_y), int(bw), int(bh)),
            (float(full_cx), float(full_cy)),
            float(area),
            float(oval_ratio),
            full_ellipse,
        )

        if best is None or candidate[0] > best[0]:
            best = candidate

    if best is None:
        return None

    full_mask = np.zeros((h, w), dtype=np.uint8)
    full_mask[y0:y1, x0:x1] = mask_roi

    hole_coords = np.array([best[2][0], best[2][1]], dtype=np.float32)

    return best, full_mask, hole_coords