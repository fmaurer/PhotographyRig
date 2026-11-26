"""Off-rig test of TMC2209Driver (run: python3 dev_tests/sim_tmc2209.py, ~2 min). Uses sim_rig."""
import os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_rig import *  # noqa: F401,F403

results = []
def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


# 1. basic pan move 1 deg, speed, setup registers
fake, bus, enc, plant, pan, tilt = rig()
check("speed from step delay (pan 80us)", abs(pan.max_dps - 2.913) < 0.01, f"{pan.max_dps:.3f} deg/s")
check("speed from step delay (tilt 350us)", abs(tilt.max_dps - 1.775) < 0.01, f"{tilt.max_dps:.3f} deg/s")
p0 = plant.out["pan"]; t0 = time.monotonic()
pan.step_to(2146)
el = time.monotonic() - t0; p1 = plant.out["pan"]
check("pan +1 deg arrives (true shaft)", abs(p1 - p0 - 1.0) < 0.1, f"moved {p1-p0:+.3f} deg in {el:.2f}s, err {pan.last_error}")
check("pan peak speed <= max", plant.peak_speed["pan"] <= pan.max_dps * 1.02, f"peak {plant.peak_speed['pan']:.2f}")
check("pan stopped + current off after move", stopped(fake, 0))
check("MRES=1/8 + mstep_reg_select set", (fake.regs[0][tu.REG_CHOPCONF] >> 24) & 0xF == 5 and fake.regs[0][tu.REG_GCONF] & 0x80)
check("step counter ~ +2146 (+-1.5 encoder counts)", abs(pan.total_signed_steps - 2146) < 2146 * 0.132, f"{pan.total_signed_steps}")
check("tilt untouched", fake.irun(1) == 0 and abs(plant.out['tilt'] + 5) < 0.01)

# 2. tilt -3 deg, then back +3 (direction + counter)
q0 = plant.out["tilt"]
tilt.step_to(-805 * 3)
q1 = plant.out["tilt"]
check("tilt -3 deg arrives (true shaft)", abs(q1 - q0 + 3) < 0.1, f"{q1-q0:+.3f}")
tilt.step_to(805 * 3)
check("tilt back to start (true shaft)", abs(plant.out["tilt"] - q0) < 0.1, f"{plant.out['tilt']-q0:+.3f}")
check("tilt counter back near 0", abs(tilt.total_signed_steps) < 805 * 0.12, f"{tilt.total_signed_steps}")
check("tilt cruise ~1.77 deg/s", plant.peak_speed["tilt"] <= 1.81 and plant.peak_speed["tilt"] > 1.6, f"{plant.peak_speed['tilt']:.2f}")

# 3. simultaneous pan + tilt, reset_step_counter
o = dict(plant.out)
th = [threading.Thread(target=pan.step_to, args=(-2146 * 2,)), threading.Thread(target=tilt.step_to, args=(805,))]
[x.start() for x in th]; [x.join() for x in th]
dp, dt_ = plant.out["pan"] - o["pan"], plant.out["tilt"] - o["tilt"]
check("simultaneous moves arrive (true shaft)", abs(dp + 2) < 0.1 and abs(dt_ - 1) < 0.1,
      f"pan {dp:+.3f} tilt {dt_:+.3f} errs {pan.last_error} {tilt.last_error}")
check("both stopped", stopped(fake, 0) and stopped(fake, 1))

# 4. refuse huge move, stop_movement mid-move, cancel+restart pattern
pan.step_to(2146 * 40)
check("40 deg move refused", "exceeds max_move_deg" in (pan.last_error or "") and stopped(fake, 0), pan.last_error)
a = enc.pos_deg("pan")
th = threading.Thread(target=pan.step_to, args=(2146 * 10,)); th.start(); time.sleep(1.5)
ts = time.monotonic(); ok = pan.stop_movement(); stop_t = time.monotonic() - ts; th.join(2)
moved = enc.pos_deg("pan") - a
check("stop_movement stops quickly", ok and stop_t < 0.6 and stopped(fake, 0) and 1.0 < moved < 6, f"{stop_t:.2f}s, moved {moved:+.2f}")
th = threading.Thread(target=pan.step_to, args=(2146 * 5,)); th.start(); time.sleep(0.8)
pan.stop_movement(); th.join(0.1)
b = enc.pos_deg("pan")
th2 = threading.Thread(target=pan.step_to, args=(-2146,)); th2.start(); th2.join(10); th.join(1)
check("cancel then reverse move arrives", abs(enc.pos_deg("pan") - b + 1) < 0.2 and stopped(fake, 0), f"{enc.pos_deg('pan')-b:+.2f}")
plant.stop = True

# 5. lash + slip: still arrives (closed loop)
fake, bus, enc, plant, pan, tilt = rig(lash={"pan": 2.3, "tilt": 0.5}, slip={"pan": 0.77, "tilt": 0.89})
for d in (+2, -2, +1):
    s = plant.out["pan"]; pan.step_to(2146 * d)
    check(f"pan {d:+d} deg with 2.3 deg lash, 0.77 slip (true shaft)", abs(plant.out["pan"] - s - d) < 0.1,
          f"{plant.out['pan']-s:+.3f} err={pan.last_error}")
plant.stop = True

# 6. frozen encoder -> follow guard; travel bounded
fake, bus, enc, plant, pan, tilt = rig()
th = threading.Thread(target=pan.step_to, args=(2146 * 8,)); th.start(); time.sleep(0.6)
m0 = plant.motor["pan"]; plant.freeze.add("pan"); th.join(15)
extra = plant.motor["pan"] - m0
check("frozen encoder stops the move", ("not following" in (pan.last_error or "") or "stuck" in (pan.last_error or "")) and stopped(fake, 0) and extra < 4.5,
      f"motor ran {extra:.2f} deg after freeze; {pan.last_error}")
plant.stop = True

# 6b. tilt encoder frozen from the start
fake, bus, enc, plant, pan, tilt = rig()
plant.freeze.add("tilt"); time.sleep(0.2)
m0 = plant.motor["tilt"]; tilt.step_to(805 * 6)
check("tilt frozen from start stops", stopped(fake, 1) and plant.motor["tilt"] - m0 < 2.5, f"ran {plant.motor['tilt']-m0:.2f}; {tilt.last_error}")
plant.stop = True

# 7. encoder disappears
fake, bus, enc, plant, pan, tilt = rig()
th = threading.Thread(target=tilt.step_to, args=(805 * 5,)); th.start(); time.sleep(0.8)
m0 = plant.motor["tilt"]; ts = time.monotonic(); plant.drop.add("tilt"); th.join(5)
check("lost encoder stops within ~0.5 s", "encoder" in (tilt.last_error or "") and stopped(fake, 1) and time.monotonic() - ts < 1.0,
      f"{time.monotonic()-ts:.2f}s ran {plant.motor['tilt']-m0:.2f} deg; {tilt.last_error}")
plant.stop = True

# 8. no encoder at all -> refused, nothing energised
fake, bus, enc, plant, pan, tilt = rig()
plant.drop.add("pan"); time.sleep(0.3)
pan.step_to(2146)
check("no encoder: move refused, motor never energised", "encoder" in (pan.last_error or "") and fake.irun(0) == 0
      and not any(w[1] == 0 and w[2] == tu.REG_VACTUAL and w[3] != 0 for w in fake.writes), pan.last_error)
plant.stop = True

# 9. driver stops answering mid-move
fake, bus, enc, plant, pan, tilt = rig()
th = threading.Thread(target=pan.step_to, args=(2146 * 6,)); th.start(); time.sleep(1.0)
fake.dead.add(0); td = time.monotonic(); th.join(8); dead_t = time.monotonic() - td
check("dead driver: move ends with an error (stop not confirmed)", pan.last_error is not None and not pan.is_moving, f"{pan.last_error} after {dead_t:.2f}s")
fake.dead.clear()
check("after recovery driver is at VACTUAL 0 (unverified zero did go out before it died? n/a) -> zero_now works",
      (bus.zero_now(), stopped(fake, 0))[1])
plant.stop = True

# 10. driver reset (motor power cycled) -> full setup redone
fake, bus, enc, plant, pan, tilt = rig()
pan.step_to(2146)
fake.regs[0][tu.REG_CHOPCONF] = 0x10000053; fake.regs[0][tu.REG_GSTAT] |= 1
pan.step_to(-2146)
check("driver reset -> MRES re-applied", (fake.regs[0][tu.REG_CHOPCONF] >> 24) & 0xF == 5)
plant.stop = True

# 11. set_max_speed via UI delay, capped
pan.set_max_speed(0.000001)
check("max speed capped at 10 deg/s", pan.max_dps == 10.0)


# 12. STOP (emergency_stop): cut immediately, no ramp; both axes
fake, bus, enc, plant, pan, tilt = rig()
ths = [threading.Thread(target=pan.step_to, args=(2146 * 8,)), threading.Thread(target=tilt.step_to, args=(805 * 8,))]
[x.start() for x in ths]; time.sleep(1.5)
before = dict(plant.motor); ts = time.monotonic()
pan.request_estop(send=False); tilt.request_estop(send=False)      # what websocket stop_all_motors does
bus.zero_now([0, 1])
okp, okt = pan.emergency_stop(requested=True), tilt.emergency_stop(requested=True); dt = time.monotonic() - ts
[x.join(2) for x in ths]
coast = {ax: abs(plant.motor[ax] - before[ax]) for ax in before}
check("STOP: both confirmed, current off, fast", okp and okt and stopped(fake, 0) and stopped(fake, 1) and dt < 1.0,
      f"{dt:.2f}s, coast pan {coast['pan']:.3f} tilt {coast['tilt']:.3f} deg; errs {pan.last_error} / {tilt.last_error}")
check("STOP: no ramp-down (coast < 0.1 deg)", max(coast.values()) < 0.1)
check("STOP: next move works", (pan.step_to(-2146), pan.last_error is None and stopped(fake, 0))[1], pan.last_error)
plant.stop = True

# 13. absolute travel limits (plant starts pan at +10 deg abs)
fake, bus, enc, plant, pan, tilt = rig(limits={"pan": [-90.0, 11.0], "tilt": [-45.0, 45.0]})
time.sleep(0.2)
check("abs_deg reads absolute position", abs(enc.abs_deg("pan") - 10.0) < 0.1, f"{enc.abs_deg('pan'):.3f}")
pan.step_to(2146 * 3)
check("move past max limit is clamped", plant.out["pan"] <= 11.0 and plant.out["pan"] > 10.5 and stopped(fake, 0),
      f"pan at {plant.out['pan']:.3f}")
pan.step_to(2146 * 2)
check("further outward move: no motion past limit", plant.out["pan"] <= 11.0, f"pan at {plant.out['pan']:.3f}; {pan.last_error}")
s0 = plant.out["pan"]; pan.step_to(-2146 * 2)
check("move back inside is allowed", abs(plant.out["pan"] - s0 + 2) < 0.1, f"{plant.out['pan']-s0:+.3f}")
plant.stop = True
fake, bus, enc, plant, pan, tilt = rig(limits={"pan": [-90.0, 11.0]}, start={"pan": 12.0, "tilt": -5.0})
time.sleep(0.2)
pan.step_to(2146)
check("starting outside: outward move refused, motor not energised", "only moves back inside" in (pan.last_error or "")
      and not any(w[1] == 0 and w[2] == tu.REG_VACTUAL and w[3] != 0 for w in fake.writes), pan.last_error)
plant.stop = True
fake, bus, enc, plant, pan, tilt = rig(limits={"pan": [-90.0, 11.0]}, slip={"pan": 1.0, "tilt": 1.0})
time.sleep(0.2)
pan.travel_limits = (-90.0, 10.8)                     # simulate the target landing right at the limit, then a runaway
th = threading.Thread(target=pan.step_to, args=(int(2146 * 0.6),)); th.start(); time.sleep(0.15)
pan.travel_limits = (-90.0, 9.9)                      # limit tightened under the move -> must stop once past 9.9 + 0.5
th.join(10)
check("crossing the limit mid-move stops", "crossed travel limit" in (pan.last_error or "") and stopped(fake, 0)
      and plant.out["pan"] < 9.9 + 0.5 + 0.15, f"pan at {plant.out['pan']:.3f}; {pan.last_error}")
plant.stop = True

print(f"\n{sum(ok for _, ok in results)}/{len(results)} passed")
