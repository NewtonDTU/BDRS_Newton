import cv2
import numpy as np


def holedet(frame):
    if frame is None:
        return None

    h, w = frame.shape[:2]

    # tuned values from test_holedet_folder.py
    lower = np.array([16, 73, 70], dtype=np.uint8)
    upper = np.array([29, 164, 164], dtype=np.uint8)

    roi_x_min = 0.00
    roi_x_max = 1.00
    roi_y_min = 0.00
    roi_y_max = 1.00

    min_area = 40
    max_area = 1800
    min_aspect = 0.45
    max_aspect = 4.39
    min_fill = 0.14
    border_margin = 7

    x0 = int(roi_x_min * w)
    x1 = int(roi_x_max * w)
    y0 = int(roi_y_min * h)
    y1 = int(roi_y_max * h)

    if x1 <= x0 or y1 <= y0:
        return None

    roi = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask_roi = cv2.inRange(hsv, lower, upper)

    contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    margin = border_margin

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw <= 0 or bh <= 0:
            continue

        if x <= margin or y <= margin or (x + bw) >= (mask_roi.shape[1] - margin) or (y + bh) >= (mask_roi.shape[0] - margin):
            continue

        aspect = bw / float(bh)
        if aspect < min_aspect or aspect > max_aspect:
            continue

        rect_area = float(bw * bh)
        fill_ratio = area / rect_area if rect_area > 1e-6 else 0.0
        if fill_ratio < min_fill:
            continue

        cx = x + bw / 2.0
        cy = y + bh / 2.0

        target_x = 0.50 * mask_roi.shape[1]
        target_y = 0.35 * mask_roi.shape[0]

        dx = abs(cx - target_x) / max(1.0, mask_roi.shape[1] / 2.0)
        dy = abs(cy - target_y) / max(1.0, mask_roi.shape[0] / 2.0)
        pos_penalty = dx + 1.2 * dy

        score = area - 300.0 * pos_penalty

        full_x = x + x0
        full_y = y + y0
        full_cx = cx + x0
        full_cy = cy + y0

        candidate = (
            score,
            (int(full_x), int(full_y), int(bw), int(bh)),
            (float(full_cx), float(full_cy)),
            float(area),
            float(aspect),
            float(fill_ratio),
        )

        if best is None or candidate[0] > best[0]:
            best = candidate

    if best is None:
        full_mask = np.zeros((h, w), dtype=np.uint8)
        return None

    full_mask = np.zeros((h, w), dtype=np.uint8)
    full_mask[y0:y1, x0:x1] = mask_roi

    hole_coords = np.array([best[2][0], best[2][1]], dtype=np.float32)
    return best, full_mask, hole_coords