#!/usr/bin/env python3
"""Reference tracking client for the PC that does the image processing.

Reads the rig's video with minimal buffering, finds the object (placeholder:
brightest blob -- replace detect() with your own), converts its pixel offset
from the image centre into pan/tilt angle errors and streams them to the rig:

    track <err_pan_deg> <err_tilt_deg>          (the rig estimates velocity)

Needs: pip install opencv-python websockets numpy

1. Calibrate once (MOVES THE RIG: pan +1 deg and back, then tilt +1 deg and
   back; point it at a textured, static scene):
       python track_client_example.py --rig samwise.local --calibrate
   Writes track_calibration.json: the 2x2 pixels-per-degree matrix (direction
   and camera roll included), so the error signs come out right.
2. Dry run (no commands sent; check the printed errors make sense):
       python track_client_example.py --rig samwise.local --show
3. Track:
       python track_client_example.py --rig samwise.local --send --latency-ms 60
   --latency-ms: frame age when its error reaches the rig (exposure to
   command). Measure it (see README notes in tracker.py) and set it; it lets
   the rig compare against where it was aiming when the frame was taken.
Ctrl-C sends track_stop. The rig's STOP button / Esc always works.
"""
import argparse
import asyncio
import json
import os
import threading
import time

# Low-latency FFmpeg input for OpenCV: UDP, no input buffer, no reordering delay.
# Must be set before cv2 opens the stream.
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                      "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0")
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import websockets  # noqa: E402

CAL_FILE = "track_calibration.json"
STEPS_PER_DEG = {"pan": 2146, "tilt": 805}     # must match main.py


class LatestFrame(threading.Thread):
    """Reads frames as fast as they arrive and keeps only the newest (no queue = no added latency)."""
    def __init__(self, url):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open {url}")
        self.frame, self.t, self.n = None, 0.0, 0
        self.cv = threading.Condition()

    def run(self):
        while True:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self.cv:
                self.frame, self.t, self.n = f, time.monotonic(), self.n + 1
                self.cv.notify_all()

    def wait_new(self, last_n, timeout=1.0):
        with self.cv:
            self.cv.wait_for(lambda: self.n != last_n, timeout)
            return self.frame, self.t, self.n


def detect(frame, thresh):
    """Placeholder detector: centroid of the brightest blob. Returns (x, y) px or None."""
    g = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (9, 9), 0)
    _, mx, _, loc = cv2.minMaxLoc(g)
    if mx < thresh:
        return None
    _, mask = cv2.threshold(g, max(thresh, mx * 0.8), 255, cv2.THRESH_BINARY)
    m = cv2.moments(mask, binaryImage=True)
    if m["m00"] == 0:
        return loc
    return (m["m10"] / m["m00"], m["m01"] / m["m00"])


def shift_px(a, b):
    """Image shift of b relative to a (px), by phase correlation."""
    ga = np.float32(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY))
    gb = np.float32(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY))
    win = cv2.createHanningWindow(ga.shape[::-1], cv2.CV_32F)
    (dx, dy), resp = cv2.phaseCorrelate(ga, gb, win)
    return dx, dy, resp


async def calibrate(ws, grab):
    """Move each axis +1 deg (UI frame) and back; measure the image shift. J[:, i] = px per deg of axis i."""
    J = np.zeros((2, 2))
    for i, ax in enumerate(("pan", "tilt")):
        _, _, n = grab.wait_new(-1)
        await asyncio.sleep(0.5)
        f0, _, n = grab.wait_new(n)
        await ws.send(f"{ax} {STEPS_PER_DEG[ax]}")
        await asyncio.sleep(3.0)
        f1, _, n = grab.wait_new(n)
        dx, dy, resp = shift_px(f0, f1)
        await ws.send(f"{ax} {-STEPS_PER_DEG[ax]}")
        await asyncio.sleep(3.0)
        print(f"{ax} +1 deg -> image shift ({dx:+.1f}, {dy:+.1f}) px, correlation {resp:.2f}")
        if resp < 0.1:
            raise SystemExit("weak correlation: point the camera at a textured, static scene and retry")
        J[:, i] = (dx, dy)
    json.dump({"J_px_per_deg": J.tolist(), "made": time.strftime("%Y-%m-%d %H:%M:%S")}, open(CAL_FILE, "w"), indent=2)
    print(f"wrote {CAL_FILE}: {J.tolist()}")


async def track(ws, grab, args):
    J = np.array(json.load(open(CAL_FILE))["J_px_per_deg"])
    Jinv = np.linalg.inv(J)
    if args.send:
        await ws.send(f"track_config latency_ms={args.latency_ms}")
        await ws.send("track_start")
        print("tracking started (Ctrl-C to stop)")
    n, t_rate, frames, sent = -1, time.monotonic(), 0, 0
    try:
        while True:
            f, t_frame, n = await asyncio.to_thread(grab.wait_new, n)
            if f is None:
                continue
            frames += 1
            p = detect(f, args.thresh)
            if p is not None:
                h, w = f.shape[:2]
                off = np.array([p[0] - w / 2.0, p[1] - h / 2.0])
                # Moving the camera by d (deg) shifts the image by J d (px). To bring the
                # object to the centre the image must shift by -off: d = -J^-1 off.
                err = -Jinv @ off
                if args.send:
                    await ws.send(f"track {err[0]:.4f} {err[1]:.4f}")
                    sent += 1
                if args.show:
                    cv2.circle(f, (int(p[0]), int(p[1])), 12, (0, 0, 255), 2)
            if args.show:
                cv2.imshow("track", f)
                if cv2.waitKey(1) == 27:
                    break
            now = time.monotonic()
            if now - t_rate >= 1.0:
                msg = f"{frames / (now - t_rate):.1f} fps, sent {sent}"
                if p is not None:
                    msg += f", err pan {err[0]:+.3f} tilt {err[1]:+.3f} deg, frame->send {1000 * (now - t_frame):.1f} ms"
                print(msg)
                t_rate, frames, sent = now, 0, 0
    finally:
        if args.send:
            await ws.send("track_stop")


async def main(args):
    grab = LatestFrame(args.stream or f"rtsp://{args.rig}:8554/cam0")
    grab.start()
    async with websockets.connect(f"ws://{args.rig}:8765", max_queue=None) as ws:
        async def drain():                       # don't let rig messages pile up
            async for m in ws:
                if '"track_error"' in m or '"fault": "' in m or "motors_stopped" in m:
                    print("rig:", m[:300])
        dr = asyncio.create_task(drain())
        try:
            if args.calibrate:
                await calibrate(ws, grab)
            else:
                await track(ws, grab, args)
        finally:
            dr.cancel()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", default="samwise.local")
    ap.add_argument("--stream", help="override, e.g. rtsp://samwise.local:8554/cam0")
    ap.add_argument("--calibrate", action="store_true", help="measure px/deg (moves the rig)")
    ap.add_argument("--send", action="store_true", help="actually send track commands (default: dry run)")
    ap.add_argument("--latency-ms", type=float, default=60.0)
    ap.add_argument("--thresh", type=float, default=200, help="detector brightness threshold (0-255)")
    ap.add_argument("--show", action="store_true")
    try:
        asyncio.run(main(ap.parse_args()))
    except KeyboardInterrupt:
        pass
