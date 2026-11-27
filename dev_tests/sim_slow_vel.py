"""Slow vel-mode smoothness (follower report 2026-11-26): at 0.15 deg/s the rig kept
the right average rate but moved in surges (+-119 % frame-to-frame, 14 % of frames
nearly stopped). Measures the simulated output shaft at 30 fps while the follower
sends vel every 50 ms, like the camera-side measurement.
Run: python3 dev_tests/sim_slow_vel.py"""
import os, statistics, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_rig import *  # noqa: F401,F403
from tracker import Tracker

results = []
def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


def run(dps, secs=8.0):
    fake, bus, enc, plant, pan, tilt = rig(lash={"pan": 1.2, "tilt": 0.05})
    pan.step_sign = tilt.step_sign = -1
    tr = Tracker({"pan": pan, "tilt": tilt}); tr.configure(boost_dps=0)
    tr.start(mode="vel", auto_release_s=2.0)
    t_end = time.monotonic() + 3.0 + secs
    t_meas = time.monotonic() + 3.0                 # settle: ramp up + gear gap taken up
    frames = []
    t_next, t_cmd = t_meas, 0.0
    while (now := time.monotonic()) < t_end:
        if now - t_cmd >= 0.05:
            tr.command_vel(dps, 0.0, 1.0); t_cmd = now
        if now >= t_next:
            frames.append((now, plant.out["pan"])); t_next += 1 / 30
        time.sleep(0.002)
    tr.command_vel(0.0, 0.0); time.sleep(0.8); tr.stop(); plant.stop = True
    sp = [abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(frames, frames[1:]) if b[0] > a[0]]
    mean = statistics.mean(sp)
    var = statistics.pstdev(sp) / mean * 100
    stall = sum(s < 0.2 * dps for s in sp) / len(sp) * 100
    return mean, var, stall, tr.fault


for dps, max_var in ((0.15, 20), (0.30, 12), (0.51, 10), (1.0, 8)):
    mean, var, stall, fault = run(dps)
    check(f"vel {dps:.2f} deg/s: steady (frame-to-frame variation <= {max_var} %, no near-stops)",
          var <= max_var and stall == 0 and abs(mean - dps) / dps < 0.10 and not fault,
          f"mean {mean:.3f} deg/s, variation +-{var:.0f} %, near-stops {stall:.0f} %"
          f"{', fault: ' + fault if fault else ''}")

print(f"\n{sum(ok for _, ok in results)}/{len(results)} passed")
sys.exit(0 if all(ok for _, ok in results) else 1)
