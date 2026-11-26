"""Off-rig test of tracker.Tracker (run: python3 dev_tests/sim_tracker.py, ~1.5 min).

Simulates the external image tracker: frames at FPS, the angle error of a
simulated object measured on the TRUE shaft position at frame time, delivered
LAT seconds later (what the PC would see), optionally with the object's
velocity as feedforward. Uses sim_rig (fake UART + gear/encoder plant)."""
import collections, math, os, random, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_rig import *  # noqa: F401,F403
from tracker import Tracker

FPS, LAT = 50.0, 0.060
results = []
def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


def setup(**kw):
    fake, bus, enc, plant, pan, tilt = rig(**kw)
    pan.step_sign = tilt.step_sign = -1          # as on the rig
    tr = Tracker({"pan": pan, "tilt": tilt})
    tr.configure(latency_ms=LAT * 1000)
    if not kw.get("lash"):                       # match the tracker's lash settings to the simulated gears
        tr.configure(boost_dps=0)
    return fake, bus, enc, plant, pan, tilt, tr


class PC(threading.Thread):
    """Fake image tracker: obj(t) -> {'pan': deg, 'tilt': deg} in the ENCODER frame (plant.out frame)."""
    def __init__(self, tr, plant, obj, ff=True, noise=0.0):
        super().__init__(daemon=True)
        self.tr, self.plant, self.obj, self.ff, self.noise = tr, plant, obj, ff, noise
        self.stop = False; self.log = []; self.t0 = time.monotonic()

    def run(self):
        q = collections.deque()
        nxt = time.monotonic()
        while not self.stop:
            now = time.monotonic()
            if now >= nxt:                                   # a frame is exposed now
                t = now - self.t0
                o = self.obj(t); o2 = self.obj(t + 0.01)
                e = {ax: (o[ax] - self.plant.out[ax]) + (random.gauss(0, self.noise) if self.noise else 0.0) for ax in o}
                v = {ax: (o2[ax] - o[ax]) / 0.01 for ax in o}
                self.log.append((t, e["pan"], e["tilt"]))
                # UI frame = step_sign * encoder frame (step_sign -1)
                if self.ff == "auto":                        # errors only: rig estimates velocity
                    q.append((now + LAT, -e["pan"], -e["tilt"], None, None))
                else:
                    q.append((now + LAT, -e["pan"], -e["tilt"], -v["pan"] if self.ff else 0.0, -v["tilt"] if self.ff else 0.0))
                nxt += 1.0 / FPS
            while q and q[0][0] <= now:
                _, ep, et, vp, vt = q.popleft()
                try:
                    self.tr.command(ep, et, vp, vt)
                except RuntimeError:
                    pass
            time.sleep(0.001)

    def rms(self, t_from, t_to=1e9, ax="pan"):
        xs = [(p if ax == "pan" else ti) for t, p, ti in self.log if t_from <= t <= t_to]
        return math.sqrt(sum(x * x for x in xs) / max(1, len(xs))) if xs else float("nan")


def start_pos(plant):
    return dict(plant.out)


# 1. start: energised, idle without commands; step_to refused while tracking
fake, bus, enc, plant, pan, tilt, tr = setup()
st = tr.start(); time.sleep(0.5)
check("start: running, energised, not moving", tr.running and fake.irun(0) > 0 and fake.irun(1) > 0
      and fake.vact(0) == 0 and fake.vact(1) == 0, st.get("fault"))
pan.step_to(2146)
check("step_to refused while tracking", "track_stop" in (pan.last_error or ""), pan.last_error)

# 2. still target 3 deg pan / -2 deg tilt away
s = start_pos(plant)
pc = PC(tr, plant, lambda t: {"pan": s["pan"] + 3.0, "tilt": s["tilt"] - 2.0}); pc.start()
time.sleep(3.0)
ep, et = s["pan"] + 3.0 - plant.out["pan"], s["tilt"] - 2.0 - plant.out["tilt"]
t_settle = next((t for t, p, ti in pc.log if abs(p) < 0.15 and abs(ti) < 0.15 and
                 all(abs(p2) < 0.15 and abs(t2) < 0.15 for tt, p2, t2 in pc.log if tt >= t)), None)
peak_over = max(-min(p for t, p, ti in pc.log), 0)
check("still target: converges", abs(ep) < 0.1 and abs(et) < 0.1, f"final err pan {ep:+.3f} tilt {et:+.3f}")
check("still target: settles < 1.5 s, overshoot < 0.3 deg", t_settle is not None and t_settle < 1.5 and peak_over < 0.3,
      f"settle {t_settle} s, overshoot {peak_over:.3f} deg")
pc.stop = True; time.sleep(0.1)

# 3. watchdog: commands stop -> motion stops, stays energised
time.sleep(0.6)
check("watchdog: no commands -> stopped, still energised", fake.vact(0) == 0 and fake.vact(1) == 0 and fake.irun(0) > 0)

# 4. moving target: sine 4 deg amplitude, 0.25 Hz (peak 6.3 deg/s): PC feedforward, rig auto-estimate, none
for ff in (True, "auto", False):
    s = start_pos(plant)
    obj = lambda t, s=s: {"pan": s["pan"] + 4 * math.sin(2 * math.pi * 0.25 * t), "tilt": s["tilt"] + 1.5 * math.sin(2 * math.pi * 0.25 * t)}
    pc = PC(tr, plant, obj, ff=ff); pc.start(); time.sleep(8.0); pc.stop = True; time.sleep(0.1)
    r_p, r_t = pc.rms(2.0), pc.rms(2.0, ax="tilt")
    lim = {True: 0.15, "auto": 0.25, False: 1.3}[ff]   # pure P lags a moving target by v/kp + v*latency
    name = {True: "PC feedforward", "auto": "rig-estimated feedforward", False: "no feedforward"}[ff]
    check(f"moving target ({name}), no lash: RMS < {lim}", r_p < lim and r_t < lim,
          f"RMS pan {r_p:.3f} tilt {r_t:.3f} deg")
st = tr.status()
check("cmd->motor latency in loop < 10 ms p95 (sim, no UART time)", st["cmd_to_motor_ms"]["p95"] is not None
      and st["cmd_to_motor_ms"]["p95"] < 10, st["cmd_to_motor_ms"])

# 5. track_stop: ramps down, de-energises, releases the axes
tr.stop()
check("track_stop: stopped, current off, axes released", not tr.running and stopped(fake, 0) and stopped(fake, 1)
      and pan.owner is None and tilt.owner is None)
plant.stop = True

# 6. lash + slip on a reversing target: boost vs no boost
res = {}
for boost in (8.0, 0.0):
    fake, bus, enc, plant, pan, tilt, tr = setup(lash={"pan": 1.2, "tilt": 0.05}, slip={"pan": 0.9, "tilt": 0.9})
    tr.configure(boost_dps=boost, lash_pan=1.2); tr.start()
    s = start_pos(plant)
    obj = lambda t, s=s: {"pan": s["pan"] + 3 * math.sin(2 * math.pi * 0.2 * t), "tilt": s["tilt"]}
    pc = PC(tr, plant, obj); pc.start(); time.sleep(10.0); pc.stop = True
    res[boost] = pc.rms(2.5); tr.stop(); plant.stop = True
check("lash 1.2 deg + slip 0.9: tracks, RMS < 0.3 (boost off and on)", res[0.0] < 0.3 and res[8.0] < 0.3,
      f"RMS boost off {res[0.0]:.3f}, boost 8 deg/s {res[8.0]:.3f}")

# 6b. measured pan lash (~2 deg): boost must not make a still target hunt, and should help a moving one
def lash_run(boost, obj_fn, secs, noise=0.02):
    fake, bus, enc, plant, pan, tilt, tr = setup(lash={"pan": 2.0, "tilt": 0.05}, slip={"pan": 0.92, "tilt": 0.9})
    tr.configure(boost_dps=boost, lash_pan=1.8); tr.start(); time.sleep(0.2)
    s = start_pos(plant)
    pc = PC(tr, plant, lambda t, s=s: obj_fn(t, s), ff="auto", noise=noise); pc.start(); time.sleep(secs); pc.stop = True
    out = pc
    tr.request_estop(); bus.zero_now(); tr.thread.join(2); plant.stop = True
    return out
for boost in (0.0, 6.0):
    pc = lash_run(boost, lambda t, s: {"pan": s["pan"] + (1.0 if t > 0.5 else 0.0), "tilt": s["tilt"]}, 7.0)
    late = [abs(p) for t, p, ti in pc.log if t > 4.0]
    check(f"still target, 2 deg lash, boost {boost:g}: no hunting (max |err| after 4 s < 0.3)", max(late) < 0.3,
          f"max {max(late):.3f} deg")
r = {}
for boost in (0.0, 6.0):
    pc = lash_run(boost, lambda t, s: {"pan": s["pan"] + math.sin(2 * math.pi * 0.25 * t), "tilt": s["tilt"]}, 9.0)
    r[boost] = pc.rms(2.0)
# The sim's dead-band lash doesn't show the benefit the rig does (live: 0.250 vs 0.325 RMS); only
# guard against the boost making things clearly worse here.
check("sine, 2 deg lash: boost not clearly worse in sim (< 1.3x)", r[6.0] < 1.3 * r[0.0],
      f"RMS boost {r[6.0]:.3f} vs off {r[0.0]:.3f} (rig: boost better)")

# 7. STOP during tracking
fake, bus, enc, plant, pan, tilt, tr = setup()
tr.start(); s = start_pos(plant)
pc = PC(tr, plant, lambda t, s=s: {"pan": s["pan"] + 3 * t, "tilt": s["tilt"]}); pc.start(); time.sleep(1.5)
m0 = plant.motor["pan"]; ts = time.monotonic()
tr.request_estop(); bus.zero_now(); tr.thread.join(2)
coast = abs(plant.motor["pan"] - m0)
check("STOP during tracking: stopped fast, released", not tr.running and stopped(fake, 0) and coast < 0.1
      and pan.owner is None and time.monotonic() - ts < 1.0, f"coast {coast:.3f} deg, fault {tr.fault}")
pc.stop = True
pan.step_to(2146)
check("after STOP: normal moves work again", pan.last_error is None, pan.last_error)
plant.stop = True

# 8. encoder freezes while tracking a moving target
fake, bus, enc, plant, pan, tilt, tr = setup()
tr.start(); s = start_pos(plant)
pc = PC(tr, plant, lambda t, s=s: {"pan": s["pan"] + 5 * t, "tilt": s["tilt"]}); pc.start(); time.sleep(1.0)
m0 = plant.motor["pan"]; plant.freeze.add("pan"); time.sleep(3.0); pc.stop = True
ran = plant.motor["pan"] - m0
check("frozen encoder while tracking: fault, stopped, bounded travel", not tr.running and stopped(fake, 0)
      and "stuck" in (tr.fault or "") and ran < 5.0, f"ran {ran:.2f} deg after freeze; {tr.fault}")
plant.stop = True

# 9. encoder disappears while tracking
fake, bus, enc, plant, pan, tilt, tr = setup()
tr.start(); s = start_pos(plant)
pc = PC(tr, plant, lambda t, s=s: {"pan": s["pan"], "tilt": s["tilt"] + 2 * t}); pc.start(); time.sleep(1.0)
plant.drop.add("tilt"); time.sleep(1.0); pc.stop = True
check("lost encoder while tracking: fault, stopped", not tr.running and stopped(fake, 1) and "lost" in (tr.fault or ""), tr.fault)
plant.stop = True

# 10. travel limit: object beyond the limit -> holds at the limit, never crosses
fake, bus, enc, plant, pan, tilt, tr = setup(limits={"pan": [-90.0, 12.0]})
tr.start(); time.sleep(0.2)
pc = PC(tr, plant, lambda t: {"pan": 20.0, "tilt": plant.out["tilt"]}); pc.start(); time.sleep(4.0); pc.stop = True
check("travel limit: stops at the limit while tracking", plant.out["pan"] <= 12.0 and plant.out["pan"] > 11.3 and tr.running,
      f"pan at {plant.out['pan']:.3f} (limit 12), fault {tr.fault}")
tr.stop(); plant.stop = True

# 11. huge/NaN errors are ignored or capped
fake, bus, enc, plant, pan, tilt, tr = setup()
tr.start(); tr.command(float("nan"), 0.0); time.sleep(0.2)
check("NaN error ignored", tr.running and fake.vact(0) == 0)
for _ in range(50):                                   # keep commanding (watchdog)
    tr.command(500.0, 0.0); time.sleep(0.02)
v = abs(tu.vactual_to_out_dps("pan", fake.vact(0)))
check("huge error: speed capped at max_dps", 0.9 * tr.cfg["max_dps"] < v <= tr.cfg["max_dps"] + 0.01, f"{v:.2f} deg/s")
tr.request_estop(); bus.zero_now(); tr.thread.join(2); plant.stop = True

# 12. over-temperature while tracking, and refusing to start hot
fake, bus, enc, plant, pan, tilt, tr = setup()
alerts = []
tr.on_alert = lambda src, m: alerts.append((src, m))
tr.start(); s = start_pos(plant)
pc = PC(tr, plant, lambda t, s=s: {"pan": s["pan"] + 3 * t, "tilt": s["tilt"]}); pc.start(); time.sleep(1.0)
fake.regs[1][tu.REG_DRV_STATUS] = 0x1                    # tilt driver hot
ts = time.monotonic(); tr.thread.join(3); pc.stop = True
check("hot while tracking: fault within ~1.1 s, stopped, alert", not tr.running and "over-temperature" in (tr.fault or "")
      and stopped(fake, 0) and stopped(fake, 1) and time.monotonic() - ts < 1.6 and alerts and alerts[-1][0] == "tracking",
      f"{time.monotonic()-ts:.2f}s {tr.fault}")
try:
    tr.start(); refused = False
except RuntimeError as e:
    refused = "over-temperature" in str(e)
check("start refused while a driver is hot, nothing energised", refused and fake.irun(0) == 0 and fake.irun(1) == 0
      and pan.owner is None)
fake.regs[1][tu.REG_DRV_STATUS] = 0
tr.start()
check("cooled: tracking starts again", tr.running)
tr.stop(); plant.stop = True

print(f"\n{sum(ok for _, ok in results)}/{len(results)} passed")
