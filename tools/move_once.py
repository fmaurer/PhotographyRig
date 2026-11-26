#!/usr/bin/env python3
"""Send ONE move to the running app (same as a UI button) and report how far
the output actually moved, read straight from the encoder.

    venv/bin/python tools/move_once.py pan 1073     # 0.5 deg at 2146 steps/deg
    venv/bin/python tools/move_once.py tilt -805    # 1 deg the other way
"""
import argparse
import asyncio
import time

import websockets
from smbus2 import SMBus

BUS = {"pan": 1, "tilt": 2}
STEPS_PER_DEG = {"pan": 2146, "tilt": 805}


def enc_deg(axis, n=15):
    with SMBus(BUS[axis]) as b:
        v = [(lambda hl: ((hl[0] << 8) | hl[1]) & 0xFFF)(b.read_i2c_block_data(0x36, 0x0E, 2)) for _ in range(n)]
    return sum(v) / len(v) * 360 / 4096


async def main(axis, steps, url, timeout):
    e0 = enc_deg(axis)
    t0 = time.monotonic()
    async with websockets.connect(url) as ws:
        await ws.send(f"{axis} {steps}")
        prev, still = e0, 0
        while time.monotonic() - t0 < timeout:
            await asyncio.sleep(0.1)
            e = enc_deg(axis)
            still = still + 1 if abs(e - prev) < 0.03 else 0
            prev = e
            if still >= 8 and abs(e - e0) > 0.05:   # moved, then settled
                break
            if still >= 30:                          # never moved
                break
    e1 = enc_deg(axis)
    d = ((e1 - e0 + 180) % 360) - 180
    print(f"{axis} {steps:+d} steps ({steps / STEPS_PER_DEG[axis]:+.3f} deg asked): encoder "
          f"{e0:.2f} -> {e1:.2f} deg, moved {d:+.3f} deg (sign depends on STEP_SIGN), "
          f"{time.monotonic() - t0:.1f} s")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("axis", choices=["pan", "tilt"])
    ap.add_argument("steps", type=int)
    ap.add_argument("--url", default="ws://localhost:8765")
    ap.add_argument("--timeout", type=float, default=20.0)
    a = ap.parse_args()
    asyncio.run(main(a.axis, a.steps, a.url, a.timeout))
