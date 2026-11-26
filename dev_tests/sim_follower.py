"""Off-rig test of the follower interface pieces (run: python3 dev_tests/sim_follower.py, ~1 min):
tracker vel mode (Tracker.command_vel), open-loop micro-nudges (TMC2209Driver),
and move start latency. Uses sim_rig (fake UART + gear/encoder plant).
Positions are judged on the TRUE output shaft (plant.out), encoder frame;
UI frame = -encoder frame (step_sign -1, as on the rig)."""
import math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_rig import *  # noqa: F401,F403
from tracker import Tracker
import TMC2209Driver as TD

results = []
def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


def setup(**kw):
    fake, bus, enc, plant, pan, tilt = rig(**kw)
    pan.step_sign = tilt.step_sign = -1
    tr = Tracker({"pan": pan, "tilt": tilt})
    tr.configure(boost_dps=0)
    return fake, bus, enc, plant, pan, tilt, tr


def stream_vel(tr, vp, vt, secs, timeout=1.0, every=0.05):
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        tr.command_vel(vp, vt, timeout)
        time.sleep(every)


def rate(plant, ax, secs=0.5):
    a = plant.out[ax]; time.sleep(secs); return (plant.out[ax] - a) / secs


# 1. vel: rate held on the shaft, other axis still, sign = UI frame
fake, bus, enc, plant, pan, tilt, tr = setup()
tr.start(mode="vel", auto_release_s=2.0)
check("vel start: running in vel mode, owner set", tr.running and tr.mode == "vel" and pan.owner == "vel mode")
t0 = plant.out["tilt"]
stream_vel(tr, 1.0, 0.0, 1.0)                    # +1 deg/s UI pan
r = None
import threading
th = threading.Thread(target=stream_vel, args=(tr, 1.0, 0.0, 1.2)); th.start()
time.sleep(0.2); r = rate(plant, "pan", 0.8); th.join()
check("vel +1 deg/s UI pan -> shaft -1 deg/s (encoder frame)", abs(r + 1.0) < 0.05, f"rate {r:+.3f}")
check("vel: tilt untouched", abs(plant.out["tilt"] - t0) < 0.02, f"{plant.out['tilt'] - t0:+.3f}")

# 2. timeout: stop sending -> stops by itself after timeout_s (+ ramp)
tr.command_vel(1.0, 0.0, 0.5)
time.sleep(0.5 + 1.0 / 60 + 0.15)
r = rate(plant, "pan", 0.3)
check("vel timeout 0.5 s: stopped by itself", abs(r) < 0.02, f"rate after {r:+.3f}")

# 3. vel 0 0: ramps down, no pull-back
th = threading.Thread(target=stream_vel, args=(tr, 3.0, 0.0, 1.0)); th.start(); th.join()
p_stop = plant.out["pan"]; tr.command_vel(0.0, 0.0, 1.0)
log = []
for _ in range(60):
    log.append(plant.out["pan"]); time.sleep(0.01)
mono = all(b <= a + 1e-3 for a, b in zip(log, log[1:]))       # encoder-frame pan was decreasing
coast = p_stop - log[-1]
check("vel 0 0: stops within ramp distance, no pull-back", mono and abs(coast) < 0.2 and abs(rate(plant, "pan", 0.2)) < 0.02,
      f"coast {coast:+.3f} deg (v^2/2a = {9/120:.3f})")

# 4. idle auto-release
time.sleep(2.4)
check("vel idle 2 s: released (not running, current off)", not tr.running and fake.irun(0) == 0 and fake.irun(1) == 0,
      f"running={tr.running} fault={tr.fault}")

# 5. vel at the rig's limits: 15 deg/s cap, ramp at 60 deg/s^2
tr.start(mode="vel", auto_release_s=2.0)
th = threading.Thread(target=stream_vel, args=(tr, 40.0, 0.0, 1.0)); th.start()
time.sleep(0.6); r = rate(plant, "pan", 0.3); th.join()
check("vel capped at max_dps 15", abs(abs(r) - 15.0) < 0.5, f"rate {r:+.2f}")
tr.command_vel(0, 0); time.sleep(0.6); tr.stop()

# 6. STOP during vel
tr.start(mode="vel", auto_release_s=2.0)
th = threading.Thread(target=stream_vel, args=(tr, 2.0, 1.0, 0.8)); th.start(); time.sleep(0.5)
tr.request_estop(); bus.zero_now(); th.join()
time.sleep(0.2)
check("STOP during vel: stopped, current off", not tr.running and fake.vact(0) == 0 and fake.irun(0) == 0
      and "STOP" in (tr.fault or ""), tr.fault)
try:
    tr.command_vel(1.0, 0.0)
    check("vel after STOP raises (server latches)", False)
except RuntimeError as e:
    check("vel after STOP raises while not running", True, str(e)[:60])
plant.stop = True

# 7. reversal across a 1.2 deg pan gap: shaft catches up with the integrated target
fake, bus, enc, plant, pan, tilt, tr = setup(lash={"pan": 1.2, "tilt": 0.0})
tr.start(mode="vel", auto_release_s=2.0)
stream_vel(tr, 0.5, 0.0, 2.0)
a = plant.out["pan"]
stream_vel(tr, -0.5, 0.0, 4.0)                   # reverse: shaft frame +0.5 deg/s for 4 s = +2 deg
moved = plant.out["pan"] - a
check("vel reversal over 1.2 deg gap: shaft travel ~ commanded (lag <= 0.5 deg)", 1.45 <= moved <= 2.1,
      f"moved {moved:+.2f} of +2.00 deg (open loop would give ~+0.8)")
tr.command_vel(0, 0); time.sleep(0.3); tr.stop(); plant.stop = True

# 8. travel limit in vel mode: brakes before the limit, no fault
fake, bus, enc, plant, pan, tilt, tr = setup(limits={"pan": (-90.0, 90.0), "tilt": (-10.0, 0.0)})
tr.start(mode="vel", auto_release_s=2.0)
stream_vel(tr, 0.0, -5.0, 2.5)                   # UI tilt -5 deg/s -> encoder +5, toward max 0
a = enc.abs_deg("tilt")
check("vel into a travel limit: stops before it (never crosses), no fault", tr.running and a is not None and a < 0.0,
      f"abs tilt {a:+.2f} (limit 0, backoff 0.2) fault={tr.fault}")
tr.command_vel(0, 0); time.sleep(0.3)

# 9. pointing moves are refused while vel mode owns the axes (the server ends vel first)
pan.step_to(2146)
check("step_to refused in vel mode", "vel mode" in (pan.last_error or ""), pan.last_error)
tr.stop(); plant.stop = True

# 10. nudges: same direction as the last turn -> open loop, exact; reversed -> closed-loop move
fake, bus, enc, plant, pan, tilt, tr = setup(lash={"pan": 1.2, "tilt": 0.0})
pan.step_to(-2146)                               # UI -1 deg -> encoder +1 deg; last_dir +1
time.sleep(0.2)
a = plant.out["pan"]
pan.step_to(-64)                                 # UI -64 steps = 0.0298 deg, encoder +, same direction
d = plant.out["pan"] - a
check("nudge same direction: open loop, ~0.030 deg", pan.last_move_kind == "nudge" and abs(d - 0.0298) < 0.004,
      f"moved {d:+.4f} kind={pan.last_move_kind} err={pan.last_error}")
a = plant.out["pan"]
pan.step_to(+64)                                 # reverse: would vanish in the gap -> closed-loop move
check("nudge reversed: runs as a closed-loop move (not open loop)", pan.last_move_kind == "move",
      f"kind={pan.last_move_kind} moved {plant.out['pan'] - a:+.4f} err={pan.last_error}")
big = 300                                        # > NUDGE_MAX_DEG (257 pan steps)
pan.step_to(-big)
check("moves above NUDGE_MAX_DEG stay closed loop", pan.last_move_kind == "move", pan.last_move_kind)
check("nudge leaves current off", fake.irun(0) == 0)
plant.stop = True

# 11. move start latency: step_to() call -> first non-zero VACTUAL write, and -> first encoder count change
fake, bus, enc, plant, pan, tilt, tr = setup()
lat_w, lat_c = [], []
for k in range(6):
    n0 = len(fake.writes)
    c0 = round(plant.out["pan"] * 4096 / 360)
    t0 = time.monotonic()
    th = threading.Thread(target=pan.step_to, args=(2146 if k % 2 else -2146,)); th.start()
    tw = tc = None
    while th.is_alive() and (tw is None or tc is None) and time.monotonic() - t0 < 3:
        if tw is None and any(w[2] == tu.REG_VACTUAL and w[3] != 0 for w in fake.writes[n0:]):
            tw = time.monotonic() - t0
        if tc is None and abs(round(plant.out["pan"] * 4096 / 360) - c0) >= 1:
            tc = time.monotonic() - t0
        time.sleep(0.001)
    th.join()
    lat_w.append(tw); lat_c.append(tc)
check("move: first VACTUAL within 0.1 s of the command (sim UART)", all(x is not None and x < 0.1 for x in lat_w),
      "ms " + ", ".join(f"{x * 1000:.0f}" for x in lat_w))
print("      move: first encoder count change after (ms): " + ", ".join(f"{x * 1000:.0f}" for x in lat_c if x))
plant.stop = True

# 12. follower move profile vs the UI profile: time and landing (true shaft), 2 deg pan, with a 1.2 deg gap
def timed(drv, plant, n, profile):
    a = plant.out["pan"]; t0 = time.monotonic()
    drv.step_to(n, profile) if profile else drv.step_to(n)
    return time.monotonic() - t0, plant.out["pan"] - a

for lash in (0.0, 1.2):
    fake, bus, enc, plant, pan, tilt, tr = setup(lash={"pan": lash, "tilt": 0.0})
    pan.step_to(2146); time.sleep(0.2)                       # take up the gap toward encoder - first
    tu_ui, d_ui = timed(pan, plant, -4292, None)             # reversal, UI profile: encoder +2 deg
    time.sleep(0.2); pan.step_to(4292); time.sleep(0.2)
    tu_f, d_f = timed(pan, plant, -4292, TD.FOLLOWER_MOVE)
    check(f"follower move faster than UI move (2 deg reversal, gap {lash})", tu_f < 0.6 * tu_ui,
          f"UI {tu_ui:.2f} s -> follower {tu_f:.2f} s")
    check(f"follower move lands within 0.1 deg (gap {lash})", abs(d_f - 2.0) <= 0.1,
          f"moved {d_f:+.3f} (UI {d_ui:+.3f}) peak {plant.peak_speed['pan']:.1f} deg/s")
    plant.stop = True

print(f"\n{sum(ok for _, ok in results)}/{len(results)} passed")
sys.exit(0 if all(ok for _, ok in results) else 1)
