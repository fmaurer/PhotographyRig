#!/usr/bin/env python3
"""Last-resort cleanup after main.py exits (main.py runs this with sudo).

REBUILT 2026-11-25: the original was only on the lost SD card. This version:
  1. disables both TMC2208 drivers (EN pins high) with `pinctrl`, which
     works even if the GPIO lines are still claimed or the process crashed;
  2. stops leftover camera / streaming processes;
  3. frees the rig's ports without killing the process that called us.
Safe to run by hand at any time: `sudo python3 force_cleanup.py`.
"""
import os
import shutil
import signal
import subprocess
import time

EN_PINS = (22, 25)          # ENABLE_PIN (pan) and ENABLE_PIN_TILT in main.py
PORTS = (8000, 8001, 8765, 8554, 8889)
PROCS = ("rpicam-vid", "rpicam-still", "ffmpeg", "mediamtx")


def motors_off():
    if not shutil.which("pinctrl"):
        print("force_cleanup: pinctrl not found; cannot force EN pins high")
        return
    for pin in EN_PINS:
        r = subprocess.run(["pinctrl", "set", str(pin), "op", "dh"], capture_output=True, text=True)
        print(f"force_cleanup: EN GPIO{pin} -> high (driver off)" if r.returncode == 0
              else f"force_cleanup: pinctrl failed on GPIO{pin}: {r.stderr.strip()}")


def pids_on_port(port):
    r = subprocess.run(["fuser", f"{port}/tcp"], capture_output=True, text=True)
    return {int(p) for p in r.stdout.split() if p.isdigit()}


def main():
    motors_off()
    spare = {os.getpid(), os.getppid()}
    for name in PROCS:
        subprocess.run(["pkill", "-TERM", "-x", name])
    victims = set()
    for port in PORTS:
        victims |= pids_on_port(port) - spare
    for pid in victims:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(1.0)
    for name in PROCS:
        subprocess.run(["pkill", "-KILL", "-x", name])
    for pid in victims:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    print(f"force_cleanup: done (signalled {sorted(victims) or 'no'} port holders)")


if __name__ == "__main__":
    main()
