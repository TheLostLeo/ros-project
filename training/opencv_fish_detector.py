#!/usr/bin/env python3
"""
Basic OpenCV fish detector (host-side, no ROS/Docker)
-----------------------------------------------------
- Reads from a webcam or a video file
- Heuristically detects fish presence using classic CV (edges/contours or HSV)
- Classifies fish "mood" as happy/disturbed using optical-flow motion metrics
- Prints results and optionally overlays on a display window

Usage examples:
  python3 training/opencv_fish_detector.py --source 0
  python3 training/opencv_fish_detector.py --source /path/to/video.mp4 --method hsv

Auto-start via teleop_web_node:
    python3 training/opencv_fish_detector.py --source 0 --send-start-on-detect \
            --teleop-url http://localhost:8080 --start-cooldown-sec 10

Keys:
    q: quit
    m: toggle detection method (shape/hsv)

Notes:
- Install deps with: pip install -r training/requirements-opencv.txt
- Ensure ffmpeg is installed so OpenCV can read most video formats.
"""
import argparse
import time
from pathlib import Path
import json
import urllib.request
import urllib.error

import cv2
import numpy as np


def open_capture(source: str):
    if source.isdigit():
        return cv2.VideoCapture(int(source))
    return cv2.VideoCapture(str(Path(source).expanduser()))


essentials = dict(
    shape_min_area=800,
    shape_min_ar=0.2,
    shape_min_solidity=0.5,
    hsv_low=(0, 30, 30),
    hsv_high=(180, 255, 255),
    hsv_min_ratio=0.015,
)


def detect_shape(bgr, shape_min_area=800, shape_min_ar=0.2, shape_min_solidity=0.5):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 40, 120)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=2)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_score = 0.0
    best_cnt = None
    for c in cnts:
        area = cv2.contourArea(c)
        if area < shape_min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        ar = min(float(w)/max(1,h), float(h)/max(1,w))
        if ar < shape_min_ar:
            continue
        hull = cv2.convexHull(c)
        hull_area = max(1.0, cv2.contourArea(hull))
        solidity = float(area) / hull_area
        if solidity < shape_min_solidity:
            continue
        score = (area / float(bgr.shape[0] * bgr.shape[1])) * ar * solidity
        if score > best_score:
            best_score = score
            best_cnt = c
    present = best_score > 0.002
    conf = float(np.clip(best_score * 50.0, 0.0, 1.0))
    return present, conf, best_cnt


def detect_hsv(bgr, hsv_low=(0, 30, 30), hsv_high=(180, 255, 255), hsv_min_ratio=0.015):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low = np.array(hsv_low, dtype=np.uint8)
    high = np.array(hsv_high, dtype=np.uint8)
    mask = cv2.inRange(hsv, low, high)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
    ratio = float(np.count_nonzero(mask)) / float(mask.size)
    present = ratio >= hsv_min_ratio
    conf = float(np.clip((ratio - hsv_min_ratio) / max(1e-6, hsv_min_ratio*3), 0.0, 1.0))
    return present, conf, mask


def draw_overlay(frame, method, present, conf, extra=None, cnt=None, mask=None, mood=None):
    h, w = frame.shape[:2]
    color = (0, 200, 0) if present else (0, 0, 200)
    text = f"method={method} present={'YES' if present else 'no'} conf={conf:.2f} mood={mood}"
    cv2.putText(frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
    if cnt is not None and len(cnt) > 0:
        cv2.drawContours(frame, [cnt], -1, color, 2)
    if mask is not None:
        small = cv2.resize(mask, (w//4, h//4))
        small_bgr = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
        frame[0:small_bgr.shape[0], w-small_bgr.shape[1]:w] = small_bgr
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', type=str, default='0', help='0 for webcam index 0, or path to video file')
    ap.add_argument('--method', type=str, default='shape', choices=['shape','hsv'], help='Detection method')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--no-display', action='store_true', help='Do not open a display window; print only')
    # Teleop integration
    ap.add_argument('--send-start-on-detect', action='store_true', help='On presence rising edge, POST start to teleop_web_node API')
    ap.add_argument('--teleop-url', type=str, default='http://localhost:8080', help='Base URL for teleop_web_node (e.g., http://host:8080)')
    ap.add_argument('--start-cooldown-sec', type=float, default=10.0, help='Cooldown seconds between start triggers')
    # Flow-based mood detection thresholds
    ap.add_argument('--flow-thr-mag', type=float, default=0.8, help='EMA mean flow magnitude threshold for disturbed')
    ap.add_argument('--flow-thr-std', type=float, default=0.7, help='EMA stddev of flow magnitude threshold for disturbed')
    ap.add_argument('--flow-thr-ratio', type=float, default=0.05, help='EMA ratio of high-motion pixels threshold for disturbed')
    ap.add_argument('--flow-pix-thr', type=float, default=1.5, help='Per-pixel magnitude threshold for counting high-motion pixels')
    ap.add_argument('--ema-sec', type=float, default=2.0, help='EMA smoothing window (seconds) for flow metrics')
    ap.add_argument('--random-mood', action='store_true', help='Fallback to random mood assignment (disable flow classification)')
    args = ap.parse_args()

    cap = open_capture(args.source)
    if not cap or not cap.isOpened():
        print(f"[ERROR] Cannot open source: {args.source}")
        return

    # Try to set capture size
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    method = args.method
    prev_gray = None
    ema_mag = 0.0
    ema_std = 0.0
    ema_ratio = 0.0
    last_ts = time.time()
    prev_present = False
    last_start_ts = 0.0
    print("Press 'm' to toggle method between shape/hsv; 'q' to quit")

    def send_start(teleop_url: str):
        url = teleop_url.rstrip('/') + '/api/cmd'
        payload = json.dumps({'cmd': 'start'}).encode('utf-8')
        req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                # Consume response to avoid ResourceWarning; ignore content
                _ = resp.read()
            print('[INFO] Sent start -> teleop_web_node')
            return True
        except urllib.error.URLError as e:
            print(f"[WARN] Failed to reach teleop API at {url}: {e}")
        except Exception as e:
            print(f"[WARN] Error calling teleop API: {e}")
        return False

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        frame = cv2.resize(frame, (args.width, args.height), interpolation=cv2.INTER_AREA)

        if method == 'hsv':
            present, conf, mask = detect_hsv(frame, essentials['hsv_low'], essentials['hsv_high'], essentials['hsv_min_ratio'])
            cnt = None
        else:
            present, conf, cnt = detect_shape(frame, essentials['shape_min_area'], essentials['shape_min_ar'], essentials['shape_min_solidity'])
            mask = None

        # Mood classification via optical flow (or random if requested)
        flow_vis = None
        if not args.random_mood:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            now = time.time()
            dt = max(1e-3, now - last_ts)
            last_ts = now
            if prev_gray is not None:
                flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None,
                                                    0.5, 3, 15, 3, 5, 1.2, 0)
                mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1], angleInDegrees=False)
                # Flow metrics
                mag_mean = float(np.mean(mag))
                mag_std = float(np.std(mag))
                high_ratio = float(np.mean(mag > args.flow_pix_thr))
                # EMA smoothing
                alpha = 1.0 - np.exp(-dt / max(1e-6, args.ema_sec))
                ema_mag = (1.0 - alpha) * ema_mag + alpha * mag_mean
                ema_std = (1.0 - alpha) * ema_std + alpha * mag_std
                ema_ratio = (1.0 - alpha) * ema_ratio + alpha * high_ratio
                disturbed = (ema_mag > args.flow_thr_mag) or (ema_std > args.flow_thr_std) or (ema_ratio > args.flow_thr_ratio)
                mood = 'disturbed' if disturbed else 'happy'
                # Optional small flow visualization
                mag_norm = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
                flow_vis = cv2.applyColorMap(mag_norm.astype(np.uint8), cv2.COLORMAP_TURBO)
            else:
                mood = 'happy'
            prev_gray = gray
        else:
            # Random fallback
            mood = 'happy' if np.random.rand() < 0.6 else 'disturbed'

        # On rising edge of presence, optionally trigger start via teleop API (with cooldown)
        if args.send_start_on_detect and present and not prev_present:
            now_ts = time.time()
            if now_ts - last_start_ts >= max(0.0, args.start_cooldown_sec):
                if send_start(args.teleop_url):
                    last_start_ts = now_ts

        prev_present = present

        print(f"present={present} conf={conf:.2f} mood={mood} (ema_mag={ema_mag:.2f} ema_std={ema_std:.2f} ema_ratio={ema_ratio:.3f})")

        if not args.no_display:
            vis = frame.copy()
            vis = draw_overlay(vis, method, present, conf, cnt=cnt, mask=mask, mood=mood)
            # Show flow magnitude map in the corner if available
            if flow_vis is not None:
                h, w = vis.shape[:2]
                small = cv2.resize(flow_vis, (w//4, h//4))
                vis[0:small.shape[0], 0:small.shape[1]] = small
            cv2.imshow('Fish Detector', vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('m'):
                method = 'hsv' if method == 'shape' else 'shape'
            elif key == ord('q'):
                break

    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
