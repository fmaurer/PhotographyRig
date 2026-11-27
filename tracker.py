"""Streaming tracking mode for the samwise rig (TMC2209 UART + AS5600 encoders).

An external tracker (image processing on another PC) streams ANGLE ERRORS at
frame rate; this loop closes on the output encoders at ~200 Hz with the motors
kept energised, so a new command changes VACTUAL within one cycle instead of
stopping, re-energising and re-accelerating like a pointing move does.

Protocol (websocket text, see websocket_server.py):
  track_start                       energise, start the loop
  track <ep> <et> [<vp> <vt>]       ep/et: angle error in degrees (object minus
                                    current aim) in the UI step frame (positive
                                    = the way a positive UI pan/tilt step moves).
                                    vp/vt: optional object angular velocity
                                    (deg/s, same frame) as feedforward. Without
                                    them the rig estimates it (auto_ff): it knows
                                    where it was aiming at each frame, so it can
                                    reconstruct the object's angle and filter its
                                    velocity (alpha-beta filter).
  track_stop                        ramp down, de-energise
  vel <pan_sps> <tilt_sps> [timeout_s]   (follower protocol, via command_vel) velocity
                                    in UI counter steps/s; the loop starts by itself
                                    (mode "vel"), ramps at accel, stops when no vel
                                    arrives for timeout_s, and releases the motors
                                    after auto_release_s idle. The rate is held on the
                                    OUTPUT shaft: the encoder target is integrated
                                    from the commanded rate (at most VEL_MAX_LAG_DEG
                                    behind/ahead of the shaft), so a gear-gap crossing
                                    on a reversal is caught up instead of lost.
  track_config key=value ...        see Tracker.CFG
  track_status                      JSON status incl. latency stats
STOP (websocket "stop" / UI button / Esc) aborts tracking immediately.

Frame latency: an error measured in a frame describes where the camera WAS
pointing when the frame was exposed. With latency_ms set to that age
(exposure end -> command arrival), the target is taken relative to the encoder
position at that time, and the object's motion over that time is added
(velocity feedforward x latency).

Safety: travel limits (driver.travel_limits, absolute), encoder lost/stuck,
driver over-temperature/short/no answer, command watchdog (no command for
watchdog_s -> stop moving, hold position energised), max speed/accel caps.
Every fault and STOP zero both drivers with unverified writes first.
"""

import collections
import math
import threading
import time

import tmc2209_uart as tu

AXES = ("pan", "tilt")


class TrackFault(RuntimeError):
    pass


class Tracker:
    CFG = {
        "kp": 6.0,              # 1/s: deg/s of correction per deg of error
        "kp_vel": 6.0,          # 1/s: the same in vel mode (on the smoothed shaft estimate)
        "max_dps": 15.0,        # per-axis speed cap (rig has run ~21 deg/s)
        "accel": 60.0,          # deg/s^2 at the output (tested in samwise's circle mode)
        "deadband": 0.05,       # deg: no correction inside this (~half an encoder count)
        "latency_ms": 0.0,      # age of the frame an error was measured in, when it arrives
        "watchdog_s": 0.3,      # no command for this long -> stop moving (hold)
        "lash_pan": 1.8,        # deg of output-equivalent motor travel to cross the gap on reversal
        "lash_tilt": 0.05,
        "lash_comp": 0.8,       # fraction of the gap crossed with the boost
        "boost_dps": 6.0,       # extra speed while crossing the gap on a reversal (0 = off).
                                # Live, pan +-1 deg sine, 3 runs each: RMS 0.250 vs 0.325 deg off,
                                # max 0.46-0.63 vs 0.76-0.83; still-target hold 0.09-0.12 vs 0.00.
                                # Only when the TARGET is moving the new way (see boost_min_ff):
                                # boosting on small corrections around a still target made the rig
                                # hunt with growing swings (live 2026-11-25).
        "boost_min_ff": 0.6,    # deg/s: boost only if the velocity feedforward is at least this, same sign.
                                # 0.3 let detector noise on a STILL target trigger it (sim: hold
                                # jitter up to 0.40 deg); 0.6: mostly 0.05-0.08, worst 0.24.
        "auto_ff": 1.0,         # 1: estimate object velocity when the command carries none
        "ff_g": 0.5,            # alpha-beta filter gains for that estimate (position, velocity)
        "ff_h": 0.2,            # (swept in sim: 0.5/0.2 best, robust to 0.02 deg detector noise)
        "ma_pan": 1000,         # run current (pan driver overheats above ~1000 mA on a bare board)
        "ma_tilt": 1400,
    }
    LOOP_DT = 0.005
    RESEND_S = 0.1
    ENC_LOST_S = 0.3
    STUCK_SLACK_DEG = 1.0
    LIMIT_BACKOFF_DEG = 0.2
    LIMIT_OVERRUN_DEG = 0.5
    HEALTH_EVERY_S = 0.5    # alternates axes: each driver checked once a second
    VEL_MAX_LAG_DEG = 0.5   # vel mode: integrated target kept within this of the shaft
    # vel mode: the speed loop works on an ESTIMATE of the shaft angle, not the raw encoder.
    # The AS5600 moves in 0.088 deg counts, so at 0.15 deg/s a count comes every ~0.6 s and
    # kp * (quantised error) swung the speed by +-0.26 deg/s: the rig surged and nearly
    # stopped a few times a second (live, follower 2026-11-26: +-119 % frame to frame).
    # Estimate = commanded motor travel (a stepper does what VACTUAL says), pulled toward
    # the encoder with this time constant; faster when they disagree by more than OBS_SNAP_DEG.
    OBS_TAU_S = 2.0
    OBS_SNAP_DEG = 0.25
    OBS_SNAP_TAU_S = 0.1

    def __init__(self, drivers):
        """drivers: {"pan": TMC2209Driver, "tilt": TMC2209Driver} sharing one bus/encoders."""
        self.drv = drivers
        self.bus = drivers["pan"].bus
        self.enc = drivers["pan"].enc
        self.cfg = dict(self.CFG)
        self.lock = threading.Lock()
        self.thread = None
        self.running = False
        self._stop = False
        self._estop = False
        self._auto_stopping = False
        self.fault = None
        self.cmd = {ax: None for ax in AXES}          # (target_enc_deg, vff_enc_dps, t_cmd, timeout_s or None)
        self.mode = None                              # "track" (track_start) or "vel" (follower)
        self.auto_release_s = None                    # vel mode: stop + release after this long idle
        self.v_now = {ax: 0.0 for ax in AXES}         # output deg/s (encoder frame) last sent
        self.t_last_cmd = None
        self.seq = 0
        self._seen_seq = 0
        self.t_recv = {}
        self.lat_ms = collections.deque(maxlen=500)   # command receipt -> VACTUAL write
        self.loop_ms = collections.deque(maxlen=500)
        self.hist = {ax: collections.deque(maxlen=400) for ax in AXES}   # (t, enc deg), ~2 s
        self.state = {}
        self.on_alert = None                          # callback(source, message) on a fault stop
        self.ab = {ax: None for ax in AXES}           # alpha-beta filter state: (t, x, v) encoder frame

    # ---- config / status ---------------------------------------------------
    def configure(self, **kw):
        for k, v in kw.items():
            if k not in self.cfg:
                raise ValueError(f"unknown track setting {k!r}; known: {', '.join(self.cfg)}")
            v = float(v)
            if v < 0:
                raise ValueError(f"{k} must be >= 0")
            if k == "max_dps":
                v = min(v, 25.0)
            if k in ("ma_pan", "ma_tilt"):
                v = min(v, 1400.0)
            self.cfg[k] = v
        return dict(self.cfg)

    def status(self):
        lat = sorted(self.lat_ms)
        loop = sorted(self.loop_ms)
        q = lambda xs, f: round(xs[min(len(xs) - 1, int(f * len(xs)))], 2) if xs else None
        age = None if self.t_last_cmd is None else round(time.monotonic() - self.t_last_cmd, 3)
        return {"running": self.running, "fault": self.fault, "last_cmd_age_s": age,
                "cmd_to_motor_ms": {"p50": q(lat, 0.5), "p95": q(lat, 0.95), "max": q(lat, 1.0)},
                "loop_ms": {"p50": q(loop, 0.5), "p95": q(loop, 0.95), "max": q(loop, 1.0)},
                "axes": self.state, "cfg": self.cfg}

    # ---- commands (called from the websocket thread) ----------------------
    def _pos_at(self, ax, t):
        """Encoder position at time t from the loop's history (linear interpolation)."""
        h = self.hist[ax]
        if not h:
            return None
        if t >= h[-1][0]:
            return h[-1][1]
        prev = None
        for tt, p in reversed(h):
            if tt <= t:
                if prev is None:
                    return p
                f = (t - tt) / (prev[0] - tt) if prev[0] > tt else 0.0
                return p + f * (prev[1] - p)
            prev = (tt, p)
        return h[0][1]

    def _ab_update(self, ax, t, x_meas):
        """Alpha-beta filter on the object's angle (encoder frame). Returns velocity (deg/s)."""
        g, h = self.cfg["ff_g"], self.cfg["ff_h"]
        st = self.ab[ax]
        if st is None or not (0.0 < t - st[0] < 0.25):      # first sample or a gap: restart
            self.ab[ax] = (t, x_meas, 0.0)
            return 0.0
        t0, x, v = st
        dt = t - t0
        x += v * dt
        r = x_meas - x
        x += g * r
        v += h * r / dt
        v = max(-self.cfg["max_dps"], min(self.cfg["max_dps"], v))
        self.ab[ax] = (t, x, v)
        return v

    def command(self, err_pan, err_tilt, v_pan=None, v_tilt=None):
        if not self.running:
            raise RuntimeError(f"tracking not running{' (fault: ' + self.fault + ')' if self.fault else ''}")
        now = time.monotonic()
        lat = self.cfg["latency_ms"] / 1000.0
        new = {}
        for ax, e, v in (("pan", err_pan, v_pan), ("tilt", err_tilt, v_tilt)):
            if e is None or not math.isfinite(e) or (v is not None and not math.isfinite(v)):
                continue
            s = self.drv[ax].step_sign
            p_then = self._pos_at(ax, now - lat)
            if p_then is None:
                continue
            obj_then = p_then + s * e                       # object angle (encoder frame) at frame time
            if v is not None:
                v_enc = s * v
                self._ab_update(ax, now - lat, obj_then)    # keep the filter warm
            elif self.cfg["auto_ff"]:
                v_enc = self._ab_update(ax, now - lat, obj_then)
            else:
                v_enc = 0.0
            # object position at frame time, moved on to now
            new[ax] = (obj_then + v_enc * lat, v_enc, now, None)
        with self.lock:
            self.cmd.update(new)
            self.t_last_cmd = now
            self.seq += 1
            self.t_recv[self.seq] = now

    def command_vel(self, v_pan, v_tilt, timeout_s=1.0):
        """Velocity command, UI frame deg/s per axis (None = leave that axis).
        Holds the rate on the output shaft (see the module doc); 0 = ramp to a
        stop without pulling back to an older position."""
        if not self.running:
            raise RuntimeError(f"tracking not running{' (fault: ' + self.fault + ')' if self.fault else ''}")
        now = time.monotonic()
        timeout_s = min(max(float(timeout_s), 0.1), 2.0)
        vmax = self.cfg["max_dps"]
        new = {}
        for ax, v in (("pan", v_pan), ("tilt", v_tilt)):
            if v is None or not math.isfinite(v):
                continue
            v_enc = self.drv[ax].step_sign * max(-vmax, min(vmax, float(v)))
            if abs(v_enc) < 1e-6:
                new[ax] = None                                  # stop: ramp down, no position hold
                continue
            p = self.hist[ax][-1][1] if self.hist[ax] else self.enc.pos_deg(ax, n=2)
            if p is None:
                continue
            prev = self.cmd.get(ax)
            if prev is not None and prev[3] is not None and now - prev[2] <= prev[3]:
                tgt = prev[0] + prev[1] * (now - prev[2])       # carry the integrated target on
            else:
                tgt = p                                         # new run: start from the shaft
            tgt = max(p - self.VEL_MAX_LAG_DEG, min(p + self.VEL_MAX_LAG_DEG, tgt))
            new[ax] = (tgt, v_enc, now, timeout_s)
        with self.lock:
            self.cmd.update(new)
            self.t_last_cmd = now
            self.seq += 1
            self.t_recv[self.seq] = now
            if self._auto_stopping:                     # idle release was ramping down: carry on instead
                self._stop = self._auto_stopping = False

    # ---- start / stop ------------------------------------------------------
    def start(self, mode="track", auto_release_s=None):
        if self.running:
            return self.status()
        for ax, d in self.drv.items():
            d.stop_movement(timeout=1.0)
            if d.is_moving:
                raise RuntimeError(f"{ax} move did not stop; not starting tracking")
        for ax, d in self.drv.items():
            if self.enc.pos_deg(ax) is None:
                raise RuntimeError(f"{ax} encoder not responding")
            self.bus.health(d.addr, ax)               # refuse to start hot / faulted / silent
        self.fault = None
        self._stop = self._estop = self._auto_stopping = False
        self.cmd = {ax: None for ax in AXES}
        self.ab = {ax: None for ax in AXES}
        self.t_last_cmd = None
        self.lat_ms.clear()
        self.loop_ms.clear()
        self.mode = mode
        self.auto_release_s = auto_release_s
        self.v_now = {ax: 0.0 for ax in AXES}
        for ax, d in self.drv.items():
            d.owner = "tracker" if mode == "track" else "vel mode"
        try:
            for ax, d in self.drv.items():
                ma = self.cfg[f"ma_{ax}"]
                if getattr(d, "_configured", False) and ma == d.current_ma and hasattr(d, "_ensure_setup"):
                    d._ensure_setup()            # already configured: just the current (much faster)
                else:
                    d.actual_ma = self.bus.setup(d.addr, ma)
                    d._configured = True
        except Exception:
            for d in self.drv.values():
                d.owner = None
            self.bus.zero_now()
            raise
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True, name="Tracker")
        self.thread.start()
        return self.status()

    def stop(self, timeout=3.0):
        """Ramp to zero, then de-energise."""
        self._stop = True
        if self.thread:
            self.thread.join(timeout)
        return self.status()

    def request_estop(self):
        """Abort now (no ramp). Caller zeroes the drivers (bus.zero_now)."""
        self._estop = True

    # ---- loop --------------------------------------------------------------
    def _run(self):
        cfg = self.cfg
        vpd = {ax: abs(tu.out_dps_to_vactual(ax, 1.0)) for ax in AXES}   # VACTUAL per output deg/s
        cur = {ax: 0 for ax in AXES}
        last_dir = {ax: 0 for ax in AXES}
        since_rev = {ax: 1e9 for ax in AXES}
        cmd_since_change = {ax: 0.0 for ax in AXES}
        last_p = {ax: None for ax in AXES}
        lost = {ax: None for ax in AXES}
        obs = {ax: None for ax in AXES}               # vel mode: smoothed shaft angle (encoder frame)
        abs_off = {}
        for ax in AXES:
            a, p = self.enc.abs_deg(ax, n=15), self.enc.pos_deg(ax, n=15)
            abs_off[ax] = None if a is None or p is None else a - p
        t_prev = t_send = t_health = t_start = time.monotonic()
        health_i = 0
        try:
            while True:
                t0 = time.monotonic()
                dt = max(t0 - t_prev, 1e-4)
                t_prev = t0
                if self._estop:
                    raise TrackFault("stopped by STOP command")
                with self.lock:
                    cmds = dict(self.cmd)
                    t_cmd = self.t_last_cmd
                    seq = self.seq
                if self.auto_release_s and not self._stop and t0 - (t_cmd or t_start) > self.auto_release_s:
                    self._stop = self._auto_stopping = True     # vel mode idle: ramp down, release
                want = {}
                for ax in AXES:
                    # commanded output travel since the last encoder change (stuck guard)
                    v_out = tu.vactual_to_out_dps(ax, cur[ax])
                    cmd_since_change[ax] += abs(v_out) * dt
                    since_rev[ax] += abs(v_out) * dt
                    p = self.enc.pos_deg(ax, n=2)
                    if p is None:
                        lost[ax] = lost[ax] or t0
                        if t0 - lost[ax] > self.ENC_LOST_S:
                            raise TrackFault(f"{ax} encoder lost")
                        want[ax] = 0.0
                        obs[ax] = None
                        continue
                    lost[ax] = None
                    self.hist[ax].append((t0, p))
                    if obs[ax] is None:
                        obs[ax] = p
                    else:
                        if since_rev[ax] >= cfg[f"lash_{ax}"]:     # inside the gear gap the shaft stays put
                            obs[ax] += v_out * dt
                        r = p - obs[ax]
                        tau = self.OBS_SNAP_TAU_S if abs(r) > self.OBS_SNAP_DEG else self.OBS_TAU_S
                        obs[ax] += r * min(dt / tau, 1.0)
                    p_ctl = obs[ax] if self.mode == "vel" else p
                    if last_p[ax] is None or abs(p - last_p[ax]) > 0.04:
                        last_p[ax], cmd_since_change[ax] = p, 0.0
                    elif cmd_since_change[ax] > cfg[f"lash_{ax}"] + self.STUCK_SLACK_DEG + 1.0:
                        raise TrackFault(f"{ax} encoder reading stuck while commanded "
                                         f"{cmd_since_change[ax]:.2f} deg")
                    c = cmds.get(ax)
                    vff = 0.0
                    expired = c is None or t0 - c[2] > (cfg["watchdog_s"] if c[3] is None else c[3])
                    if self._stop or expired:
                        v = 0.0
                        e = None
                    else:
                        tgt, vff, tc, _ = c
                        tgt_now = tgt + vff * (t0 - tc)
                        e = tgt_now - p_ctl
                        v = vff + cfg["kp_vel" if self.mode == "vel" else "kp"] * e
                        if abs(e) < cfg["deadband"] and abs(vff) < 0.05:
                            v = 0.0
                    vmax = cfg["max_dps"]
                    v = max(-vmax, min(vmax, v))
                    # absolute travel limits
                    lim = self.drv[ax].travel_limits
                    if lim and abs_off[ax] is not None:
                        a = p + abs_off[ax]
                        lo, hi = lim
                        if a > hi + self.LIMIT_OVERRUN_DEG or a < lo - self.LIMIT_OVERRUN_DEG:
                            raise TrackFault(f"{ax} crossed travel limit [{lo:g}, {hi:g}] at {a:.2f} deg")
                        # brake so we can stop before the limit (v^2 = 2 a d)
                        if v > 0:
                            v = min(v, math.sqrt(2.0 * cfg["accel"] * max(hi - self.LIMIT_BACKOFF_DEG - a, 0.0)))
                        elif v < 0:
                            v = max(v, -math.sqrt(2.0 * cfg["accel"] * max(a - (lo + self.LIMIT_BACKOFF_DEG), 0.0)))
                    # backlash: on a reversal, cross the gap with a boost
                    d = (v > 0.05) - (v < -0.05)
                    if d and d != last_dir[ax]:
                        if last_dir[ax]:
                            since_rev[ax] = 0.0
                        last_dir[ax] = d
                    gap = cfg[f"lash_{ax}"] * cfg["lash_comp"]
                    if (d and since_rev[ax] < gap and cfg["boost_dps"] > 0
                            and abs(vff) >= cfg["boost_min_ff"] and (vff > 0) == (d > 0)):
                        v = max(-vmax, min(vmax, v + d * cfg["boost_dps"]))   # max_dps stays a hard cap
                    want[ax] = v
                    self.state[ax] = {"pos": round(p, 3), "est": round(p_ctl, 3), "err": None if e is None else round(e, 3),
                                      "v_cmd": round(v, 2), "abs": None if abs_off[ax] is None else round(p + abs_off[ax], 2)}
                # acceleration limit, in VACTUAL units
                new = {}
                for ax in AXES:
                    target_v = tu.out_dps_to_vactual(ax, want[ax])
                    step = cfg["accel"] * vpd[ax] * dt
                    new[ax] = int(round(cur[ax] + max(-step, min(step, target_v - cur[ax]))))
                    v_out = tu.vactual_to_out_dps(ax, new[ax])
                    self.v_now[ax] = v_out
                    if abs(v_out) > 1e-3:
                        self.drv[ax].last_dir = 1 if v_out > 0 else -1   # motor side of the gear gap (nudges)
                if new != cur or t0 - t_send >= self.RESEND_S:
                    self.bus.w_many_fast([(self.drv[ax].addr, tu.REG_VACTUAL, new[ax]) for ax in AXES])
                    t_send = time.monotonic()
                # command receipt -> the cycle that acted on it (its VACTUAL written, if it changed anything)
                if seq != self._seen_seq:
                    t_done = time.monotonic()
                    for s_ in range(self._seen_seq + 1, seq + 1):
                        tr = self.t_recv.pop(s_, None)
                        if tr is not None:
                            self.lat_ms.append((t_done - tr) * 1000.0)
                    self._seen_seq = seq
                cur = new
                if self._stop and all(v == 0 for v in cur.values()):
                    break
                if t0 - t_health >= self.HEALTH_EVERY_S:
                    t_health = t0
                    ax = AXES[health_i % 2]
                    health_i += 1
                    try:
                        self.bus.health(self.drv[ax].addr, ax)
                        self._health_fail = None
                    except IOError:
                        first = getattr(self, "_health_fail", None) or t0
                        self._health_fail = first
                        if t0 - first > 1.5:
                            raise TrackFault(f"{ax} driver stopped answering on the UART")
                    except RuntimeError as e:
                        raise TrackFault(str(e))
                self.loop_ms.append((time.monotonic() - t0) * 1000.0)
                rest = self.LOOP_DT - (time.monotonic() - t0)
                if rest > 0:
                    time.sleep(rest)
        except Exception as e:
            self.fault = str(e)
            self.bus.zero_now()                         # SAFETY: unverified stop first, no reads
            print(f"Tracker: stopped: {e}", flush=True)
            if self.on_alert and "STOP command" not in str(e):
                try:
                    self.on_alert("tracking", str(e))
                except Exception:
                    pass
        finally:
            for ax, d in self.drv.items():
                try:
                    self.bus.stop_verified(d.addr, release=True)
                except Exception as e:
                    self.fault = (self.fault or "") + f" | {ax} stop not confirmed: {e} -- CUT MOTOR POWER"
                    print(f"Tracker: {ax} stop not confirmed -- CUT MOTOR POWER", flush=True)
                d.owner = None
            self.running = False
            print(f"{time.strftime('%H:%M:%S')} Tracker ({self.mode}) ended"
                  f"{': ' + self.fault if self.fault else (' (idle release)' if self._auto_stopping else '')}", flush=True)
            self._stop = self._auto_stopping = False
            self.v_now = {ax: 0.0 for ax in AXES}
