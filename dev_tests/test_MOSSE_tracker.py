#!/usr/bin/env python3
"""
Plane detector/tracker demo
- Green bbox around sky object
- Red dot at target center
- Loops video and aims for realtime playback

Usage:
  python test_MOSSE_tracker.py --video path/to/video.mp4
"""

import time
import argparse
import cv2
import numpy as np

# --- Detection: white + low-saturation + gentle high-pass, then contours ---
def detect_plane_bbox(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # Plane tends to be bright and low-saturation (nearly white)
    whiteish = (s < 60) & (v > 170)

    # Kill smooth sky; keep plane edges
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    hp = cv2.subtract(gray, cv2.GaussianBlur(gray, (0, 0), 7))
    edges = hp > 2

    mask = (whiteish & edges).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        area = w * h
        if area < 120:
            continue
        ar = max(w, h) / max(1, min(w, h))  # elongation
        if 1.2 <= ar <= 10 and area > best_area:
            best_area = area
            best = (x, y, w, h)
    return best

def create_mosse():
    # Works with both OpenCV API layouts
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMOSSE_create"):
        return cv2.legacy.TrackerMOSSE_create()
    if hasattr(cv2, "TrackerMOSSE_create"):
        return cv2.TrackerMOSSE_create()
    raise RuntimeError("MOSSE tracker not available. Install opencv-contrib-python")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, help="Path to MP4")
    ap.add_argument("--scale", type=float, default=1.0, help="Resize factor (e.g. 0.75 for speed)")
    ap.add_argument("--detect-every", type=int, default=5, help="Run detector every N frames")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Could not open {args.video}")

    # Target FPS; fall back to 30 if unknown
    src_fps = cap.get(cv2.CAP_PROP_FPS)
    fps = src_fps if src_fps and src_fps > 1 else 30.0
    frame_period = 1.0 / fps

    tracker = None
    frame_idx = 0
    win = "plane_track (q=quit, space=pause)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    paused = False
    t_smooth = None

    while True:
        t0 = time.time()

        if not paused:
            ok, frame = cap.read()
            if not ok:
                # loop
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                frame_idx = 0
                tracker = None
                continue

            if args.scale != 1.0:
                frame = cv2.resize(frame, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)

            # Re-detect periodically or if we lost the tracker
            need_detect = (frame_idx % args.detect_every == 0) or (tracker is None)
            bbox = None

            if need_detect:
                bbox = detect_plane_bbox(frame)
                if bbox is not None:
                    tracker = create_mosse()
                    tracker.init(frame, tuple(map(float, bbox)))  # MOSSE expects floats
            else:
                # Update tracker
                ok_trk, tb = tracker.update(frame)
                if ok_trk:
                    bbox = tuple(map(int, tb))
                else:
                    tracker = None  # force re-detect next loop

            # Draw overlays
            if bbox is not None:
                x, y, w, h = bbox
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)      # green box
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1, lineType=cv2.LINE_AA)  # red dot

            # FPS overlay
            t1 = time.time()
            inst = 1.0 / max(1e-6, (t1 - t0))
            t_smooth = inst if t_smooth is None else (0.9 * t_smooth + 0.1 * inst)
            cv2.putText(frame, f"FPS {t_smooth:4.1f}  src {fps:.1f}",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2, cv2.LINE_AA)

            cv2.imshow(win, frame)
            frame_idx += 1

            # Aim for realtime: wait the remaining budget (if any)
            elapsed = time.time() - t0
            wait_ms = max(1, int(1000 * max(0.0, frame_period - elapsed)))
        else:
            # paused: just wait
            wait_ms = 30

        key = cv2.waitKey(wait_ms) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            paused = not paused

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
