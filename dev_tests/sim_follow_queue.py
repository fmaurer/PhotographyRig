"""Reproduces the follower report of 2026-11-26 off-rig: after a slew, pan ignored
vel/move for ~5 s, and steering moves were carried out late, all at once.
Drives the REAL server code paths (WebSocketServer._start_axis_sync / _vel_job)
on the simulated rig, with the follower's timing:
  slew move -> launch ramp of small moves every 60 ms -> vel every 50 ms
  (case V), or relative moves every 0.15 s (case M).
Run: python3 dev_tests/sim_follow_queue.py"""
import asyncio, concurrent.futures, os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_rig import *  # noqa: F401,F403
from tracker import Tracker
import TMC2209Driver as TD
import websocket_server as WS

results = []
def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


class MC:                                   # motor_controller stand-in (the server uses .driver / .step_to)
    def __init__(self, d): self.driver = d
    def step_to(self, n): self.driver.step_to(n)


def server_for(pan, tilt):
    s = WS.WebSocketServer.__new__(WS.WebSocketServer)        # no cameras, sockets or threads
    s.motor_controller, s.motor_controller_tilt = MC(pan), MC(tilt)
    s.tracker = Tracker({"pan": pan, "tilt": tilt}); s.tracker.configure(boost_dps=0)
    s.current_pan_movement = s.current_tilt_movement = None
    s.movement_lock = threading.Lock()
    s._motion_exec = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    s._stop_gen = 0; s._move_seq = {}; s._vel_pending = None; s._vel_starting = False
    s._vel_block = None; s._vel_fault_ack = None; s._vel_last_rx = 0.0
    s._err_sent = {}
    s.errors = []
    async def _err(ws, text, every_s=1.0): s.errors.append((time.monotonic(), text))
    s._send_error_throttled = _err
    return s


def move(s, dp, dt):                         # what the "move" handler does (fire and forget, ordered)
    prof = dict(TD.FOLLOWER_MOVE)
    for ax, n in (("pan", dp), ("tilt", dt)):
        seq = s._move_seq[ax] = s._move_seq.get(ax, 0) + 1
        s._motion_exec.submit(s._start_axis_sync, ax, n, s._stop_gen, prof, seq)


def vel(s, sp, st, loop):                    # what the "vel" handler does
    vp, vt = sp / 2146.0, st / 805.0
    s._vel_pending = (vp, vt, 1.0)
    if s.tracker.running and s.tracker.mode == "vel":
        s.tracker.command_vel(vp, vt, 1.0)
    elif not s._vel_starting:
        s._vel_starting = True
        s._motion_exec.submit(s._vel_job, None, loop, s._stop_gen)


def wait_idle(pan, tilt, timeout=10):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if not pan.is_moving and not tilt.is_moving:
            time.sleep(0.05)
            if not pan.is_moving and not tilt.is_moving:
                return
        time.sleep(0.01)


def run(case):
    fake, bus, enc, plant, pan, tilt = rig(lash={"pan": 1.2, "tilt": 0.05})
    pan.step_sign = tilt.step_sign = -1
    s = server_for(pan, tilt)
    loop = asyncio.new_event_loop(); threading.Thread(target=loop.run_forever, daemon=True).start()
    move(s, -19699, 652)                                     # slew (follow B), from rest
    time.sleep(0.2); wait_idle(pan, tilt)
    p0 = plant.out["pan"]; t_land = time.monotonic()
    ramp = [-31, -50, -69, -93, -123, -154, -188, -230, -274, -318]
    for d in ramp:                                           # launch ramp: small moves every 60 ms
        move(s, d, 0); time.sleep(0.06)
    t_steer = time.monotonic()
    first = None; sent_pan = sum(ramp)
    t_end = t_steer + 6.0
    while time.monotonic() < t_end:
        if case == "V":
            vel(s, -2000, 0, loop); time.sleep(0.05)         # pan -2000 sps = UI -0.93 deg/s
        else:
            move(s, -500, 0); sent_pan += -500; time.sleep(0.15)
        if first is None and abs(plant.out["pan"] - p0) > 0.03:          # shaft visibly moving (~64 steps)
            first = time.monotonic() - t_steer
    if case == "V":
        vel(s, 0, 0, loop)
    time.sleep(0.5); wait_idle(pan, tilt, 15)
    if s.tracker.running:
        s.tracker.stop()
    moved_steps = (plant.out["pan"] - p0) * 2146 * -1        # UI steps (step_sign -1)
    plant.stop = True
    return first, moved_steps, sent_pan, s.errors


f, moved, sent, errs = run("V")
check("vel after slew + launch ramp: pan moves within 0.6 s of the first vel",
      f is not None and f < 0.6, f"first pan motion after {f if f is None else round(f, 2)} s; "
      f"{len(errs)} vel errors{(': ' + errs[0][1][:70]) if errs else ''}")
f, moved, sent, errs = run("M")
check("move steering after slew: pan moves within 0.6 s", f is not None and f < 0.6,
      f"first pan motion after {f if f is None else round(f, 2)} s")
check("move steering: total travel no more than what was sent (latest move wins, none replayed late)",
      abs(moved) <= abs(sent) + 400, f"moved {moved:+.0f} steps, sent {sent:+d} (incl. ramp)")

print(f"\n{sum(ok for _, ok in results)}/{len(results)} passed")
sys.exit(0 if all(ok for _, ok in results) else 1)
