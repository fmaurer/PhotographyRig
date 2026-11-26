#!/usr/bin/env python3
"""Show or set samwise's absolute travel limits (rig_limits.json).

Angles are output degrees from encoder_zero_raw, read straight from the
AS5600s (safe while the app runs; it only reads). The app loads the file at
startup, so restart it after changing anything.

    python3 tools/travel_limits.py                 # current angles + limits
    python3 tools/travel_limits.py --set-zero pan  # current pan position becomes 0 deg
    python3 tools/travel_limits.py --set pan -120 95
    python3 tools/travel_limits.py --here pan min  # current pan position becomes the min limit
    python3 tools/travel_limits.py --disable tilt
"""
import argparse
import json
import os

from smbus2 import SMBus

PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rig_limits.json")
BUS = {"pan": 1, "tilt": 2}


def raw(axis, n=20):
    with SMBus(BUS[axis]) as b:
        v = [(lambda hl: ((hl[0] << 8) | hl[1]) & 0xFFF)(b.read_i2c_block_data(0x36, 0x0E, 2)) for _ in range(n)]
    return round(sum(v) / len(v))


def deg_from_zero(r, z):
    return (((r - z + 2048) % 4096) - 2048) * 360.0 / 4096.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set-zero", choices=list(BUS))
    ap.add_argument("--set", nargs=3, metavar=("AXIS", "MIN", "MAX"))
    ap.add_argument("--here", nargs=2, metavar=("AXIS", "min|max"))
    ap.add_argument("--disable", choices=list(BUS))
    a = ap.parse_args()

    with open(PATH) as f:
        cfg = json.load(f)
    zero = cfg.setdefault("encoder_zero_raw", {})
    lim = cfg.get("limits_deg") or {}
    cfg["limits_deg"] = lim
    changed = False

    if a.set_zero:
        old = zero.get(a.set_zero)
        zero[a.set_zero] = raw(a.set_zero)
        if old is not None and lim.get(a.set_zero):
            shift = deg_from_zero(zero[a.set_zero], old)   # keep limits at the same physical place
            lim[a.set_zero] = [round(v - shift, 2) for v in lim[a.set_zero]]
        changed = True
    if a.set:
        ax, lo, hi = a.set[0], float(a.set[1]), float(a.set[2])
        if ax not in BUS or lo >= hi:
            ap.error("--set AXIS MIN MAX with MIN < MAX")
        lim[ax] = [lo, hi]
        changed = True
    if a.here:
        ax, which = a.here
        if ax not in BUS or which not in ("min", "max") or ax not in zero:
            ap.error("--here AXIS min|max (axis needs an encoder zero)")
        here = round(deg_from_zero(raw(ax), zero[ax]), 2)
        cur = lim.get(ax) or [-180.0, 180.0]
        cur[0 if which == "min" else 1] = here
        if cur[0] >= cur[1]:
            ap.error(f"that would make min >= max: {cur}")
        lim[ax] = cur
        changed = True
    if a.disable:
        lim[a.disable] = None
        changed = True

    if changed:
        with open(PATH, "w") as f:
            json.dump(cfg, f, indent=2)
            f.write("\n")
        print(f"wrote {PATH} -- restart the app to apply")
    for ax in BUS:
        r = raw(ax)
        z = zero.get(ax)
        pos = "n/a (no zero)" if z is None else f"{deg_from_zero(r, z):+.2f} deg"
        print(f"{ax}: raw {r}, position {pos}, limits {lim.get(ax) or 'none'}")


if __name__ == "__main__":
    main()
