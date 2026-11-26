"""TMC2209 (UART/VACTUAL) driver for the samwise rig, closed loop on the AS5600
output-shaft encoders. Drop-in for TMC2208Driver as far as websocket_server.py
and astro_controller.py use it.

Units: the UI and pointing calibration talk in "steps". Here a step is a
virtual unit of 1/steps_per_deg output degrees (defaults pan 2146, tilt 805,
the numbers index.html's click-to-point uses), so existing step-based UI
values keep their angular meaning. step_delay keeps its old meaning too:
speed = 1 / (2 * step_delay) steps/s, converted to output deg/s.

How a move works: read the output encoder, target = start + delta, drive
VACTUAL with an acceleration-limited profile that slows down near the target,
stop when within tolerance (or just past it: no hunting across the gear gap),
then confirm VACTUAL=0 and (by default) cut the current. Because the encoder
is on the output shaft, gear backlash and the slip measured on this rig are
corrected by the loop; browser-side backlash compensation is not needed.

Safety (each one stops the move and the motor):
  * stop_movement() / cancel from a newer move
  * travel guard: output more than |move| + TRAVEL_MARGIN_DEG from the start
  * follow guard: commanded travel exceeds measured travel by the gear lash
    + slack (stall, slipping coupling)
  * stuck guard: reading hasn't changed while the commanded travel since its
    last change exceeds the gear lash + 1 deg (frozen encoder)
  * encoder missing > 0.3 s, driver over-temperature/short/no answer, timeout
  * moves larger than max_move_deg are refused; speed capped at MAX_DPS
  * optional absolute travel limits (rig_limits.json): targets clamped,
    stop if crossed
Every exit path sends VACTUAL=0 (unverified first, then verified).

Not supported: backlash_calibration.py's raw step access (step_motor,
enable_device, dir_device). It measures backlash in open-loop steps, which
this closed-loop driver doesn't have.
"""

import math
import threading
import time

from motor_driver import MotorDriver
import tmc2209_uart as tu

MAX_DPS = 10.0                 # hard cap on output speed (deg/s); rig tested to ~21 deg/s
LASH_DEG = {"pan": 2.5, "tilt": 0.8}   # measured play at the output (pan ~2.3, tilt ~0.4-1.3)
TRAVEL_MARGIN_DEG = 3.0
LIMIT_OVERRUN_DEG = 0.5        # absolute travel limit: stop if crossed by this much
LIMIT_BACKOFF_DEG = 0.2        # clamp targets this far inside the limit
FOLLOW_SLACK_DEG = 1.0
FOLLOW_SLACK_FRAC = 0.25       # covers the approximate GEAR ratio and slip (0.76-0.97x measured)
STUCK_SLACK_DEG = 1.0          # stuck guard: commanded travel since the reading last changed > lash + this
ENC_LOST_S = 0.3
HEALTH_EVERY_S = 1.0
RESEND_S = 0.2                 # re-send unchanged VACTUAL this often (unverified writes can drop)
LOOP_DT = 0.005


class MoveError(RuntimeError):
    pass


class TMC2209Driver(MotorDriver):
    def __init__(self, bus, encoders, axis, step_delay, steps_per_deg,
                 current_ma=1000, step_sign=+1, soft_limits=None,
                 max_move_deg=30.0, accel_dps2=20.0, min_dps=0.6, kp=3.0,
                 tol_deg=0.05, hold_after_move=False, travel_limits=None):
        """bus: tmc2209_uart.TMC2209Bus (shared). encoders: AS5600Encoders (shared).
        axis: 'pan' or 'tilt'. step_sign: +1 if a positive UI step should
        increase the encoder angle, else -1 (check on the first move).
        soft_limits: (min_steps, max_steps) from the counter zero, or None.
        travel_limits: (min_deg, max_deg) ABSOLUTE output angle (encoders.abs_deg,
        survives restarts), or None. Moves are clamped to it, a move that starts
        outside may only go back in, and the loop stops if it's crossed."""
        if axis not in tu.ADDR:
            raise ValueError(f"axis must be one of {list(tu.ADDR)}")
        self.bus, self.enc, self.axis = bus, encoders, axis
        self.addr = tu.ADDR[axis]
        self.steps_per_deg = float(steps_per_deg)
        self.step_sign = 1 if step_sign >= 0 else -1
        self.current_ma = current_ma
        self.max_move_deg = float(max_move_deg)
        self.accel = float(accel_dps2)
        self.min_dps = float(min_dps)
        self.kp = float(kp)
        self.tol = float(tol_deg)
        self.hold_after_move = hold_after_move
        self.soft_limits = tuple(soft_limits) if soft_limits else None
        self.travel_limits = tuple(travel_limits) if travel_limits else None

        self.base_delay = step_delay
        self.max_dps = self._delay_to_dps(step_delay)

        self.is_moving = False
        self.should_stop = False            # kept for compatibility; stop uses a generation counter
        self.ramping_enabled = True
        self.ramp_settings = {'accel_percent': 0.3, 'max_accel_steps': 50}
        self.last_error = None
        self.last_miss_deg = None

        self._gen = 0
        self._estop = False
        self.owner = None                   # "tracker" while tracking mode drives this axis
        self._move_lock = threading.Lock()
        self._configured = False
        self._zero_deg = None
        self._last_steps = 0
        self._cur = 0                       # VACTUAL last sent

    # ---- units ------------------------------------------------------------
    def _delay_to_dps(self, delay):
        if delay <= 0:
            raise ValueError("step delay must be > 0")
        return min(MAX_DPS, 1.0 / (2.0 * delay * self.steps_per_deg))

    def num_steps_for_angle(self, degrees):
        return round(degrees * self.steps_per_deg)

    # ---- position (encoder based) -----------------------------------------
    def _pos(self, n=3):
        return self.enc.pos_deg(self.axis, n=n)

    @property
    def total_signed_steps(self):
        p = self._pos()
        if p is not None:
            if self._zero_deg is None:
                self._zero_deg = p
            self._last_steps = int(round((p - self._zero_deg) * self.steps_per_deg * self.step_sign))
        return self._last_steps

    @total_signed_steps.setter
    def total_signed_steps(self, value):
        self.reset_step_counter(value)

    def reset_step_counter(self, value=0):
        p = self._pos()
        if p is None:
            raise RuntimeError(f"{self.axis} encoder not responding; can't zero the step counter")
        self._zero_deg = p - self.step_sign * float(value) / self.steps_per_deg
        self._last_steps = int(value)

    def set_soft_limits(self, min_steps=None, max_steps=None):
        self.soft_limits = None if min_steps is None and max_steps is None else (
            -math.inf if min_steps is None else min_steps,
            math.inf if max_steps is None else max_steps)

    def set_approach(self, direction=0, overshoot_steps=0):
        """No-op: the loop closes on the output encoder, so lash is already taken up."""
        return

    # ---- moves ------------------------------------------------------------
    def step_to_angle(self, degrees):
        self.step_to(self.num_steps_for_angle(degrees))
        return 0

    def step_to(self, number):
        """Relative move of `number` signed steps (name is historical)."""
        number = int(number)
        if number == 0:
            return
        with self._move_lock:
            gen = self._gen
            self._estop = False
            try:
                self._move(number, gen)
                self.last_error = None
            except Exception as e:
                self.last_error = str(e)
                print(f"TMC2209 {self.axis}: move stopped: {e}")

    def _clamp_steps(self, number):
        if not self.soft_limits:
            return number
        lo, hi = self.soft_limits
        now = self.total_signed_steps
        tgt = min(max(now + number, lo), hi)
        if tgt != now + number:
            print(f"TMC2209 {self.axis}: move of {number} would reach {now + number}, outside soft limits "
                  f"[{lo}, {hi}]; clamped to {tgt}")
        return int(tgt - now)

    def _send(self, x):
        x = int(round(x))
        self.bus.w_fast(self.addr, tu.REG_VACTUAL, x)
        self._cur = x
        return x

    def _ensure_setup(self):
        """Full setup after power-up or a driver reset, otherwise just restore the current."""
        if self._configured:
            gs = self.bus.r(self.addr, tu.REG_GSTAT)
            if gs & 0x1:                     # driver was reset (motor power cycled): registers are back to defaults
                self._configured = False
        if not self._configured:
            self.actual_ma = self.bus.setup(self.addr, self.current_ma)
            self._configured = True
        else:
            cs, _ = tu.current_to_cs(self.current_ma)
            if not self.bus.w(self.addr, tu.REG_IHOLD_IRUN, (cs // 2) | (cs << 8) | (6 << 16)):
                raise MoveError("current write not acknowledged")

    def _move(self, number, gen):
        if self.owner:
            raise MoveError(f"{self.owner} is driving {self.axis}; send track_stop first")
        number = self._clamp_steps(number)
        if number == 0:
            return
        deg = self.step_sign * number / self.steps_per_deg       # encoder-frame output degrees
        if abs(deg) > self.max_move_deg:
            raise MoveError(f"move of {deg:+.2f} deg exceeds max_move_deg {self.max_move_deg:g}; refused")
        start = self._pos(n=15)              # ~75 ms average: start error is part of the landing error
        if start is None:
            raise MoveError(f"{self.axis} encoder not responding; moves need it")
        if self._zero_deg is None:
            self._zero_deg = start
        abs_off = None                       # absolute angle = encoder-frame angle + abs_off
        if self.travel_limits:
            a = self.enc.abs_deg(self.axis, n=15)
            if a is None:
                raise MoveError(f"{self.axis} travel limits set but no absolute angle (encoder zero missing?)")
            abs_off = a - start
            lo, hi = self.travel_limits
            a_tgt = a + deg
            if a_tgt > hi - LIMIT_BACKOFF_DEG or a_tgt < lo + LIMIT_BACKOFF_DEG:
                if a > hi - LIMIT_BACKOFF_DEG and deg > 0 or a < lo + LIMIT_BACKOFF_DEG and deg < 0:
                    raise MoveError(f"{self.axis} at {a:.2f} deg, at/outside travel limits [{lo:g}, {hi:g}]; "
                                    f"only moves back inside are allowed")
                clamped = min(max(a_tgt, lo + LIMIT_BACKOFF_DEG), hi - LIMIT_BACKOFF_DEG)
                print(f"TMC2209 {self.axis}: target {a_tgt:.2f} deg outside travel limits [{lo:g}, {hi:g}]; "
                      f"clamped to {clamped:.2f}")
                deg = clamped - a
                if abs(deg) < self.tol:
                    return
        target = start + deg
        vmax = max(self.max_dps, self.min_dps)
        acc_v = abs(tu.out_dps_to_vactual(self.axis, self.accel))   # VACTUAL units per second
        timeout = abs(deg) / vmax * 2.0 + 3.0 + vmax / self.accel
        travel_lim = abs(deg) + TRAVEL_MARGIN_DEG
        follow_slack = LASH_DEG[self.axis] + FOLLOW_SLACK_DEG

        self.is_moving = True
        self._cur = 0
        t0 = last = last_send = last_health = time.monotonic()
        lost_since = health_fail_since = None
        cmd_travel = 0.0                     # integrated commanded output travel (deg, encoder frame)
        last_p, cmd_at_change = None, 0.0    # stuck guard
        try:
            self._ensure_setup()
            while True:
                now = time.monotonic()
                dt = max(now - last, 1e-3)
                last = now
                cmd_travel += tu.vactual_to_out_dps(self.axis, self._cur) * dt
                if gen != self._gen:
                    if self._estop:
                        raise MoveError("stopped by STOP command")
                    break                    # stop_movement() / superseded: ramp down
                if now - t0 > timeout:
                    raise MoveError(f"timed out after {timeout:.1f} s")
                p = self._pos()
                if p is None:
                    lost_since = lost_since or now
                    if now - lost_since > ENC_LOST_S:
                        raise MoveError(f"{self.axis} encoder lost")
                    want = 0.0
                else:
                    lost_since = None
                    if abs_off is not None:
                        lo, hi = self.travel_limits
                        a = p + abs_off
                        if a > hi + LIMIT_OVERRUN_DEG or a < lo - LIMIT_OVERRUN_DEG:
                            raise MoveError(f"{self.axis} crossed travel limit [{lo:g}, {hi:g}] at {a:.2f} deg")
                    if abs(p - start) > travel_lim:
                        raise MoveError(f"travel guard: {p - start:+.2f} deg from start (move {deg:+.2f})")
                    along = (p - start) * math.copysign(1, cmd_travel or deg)
                    if abs(cmd_travel) - along > follow_slack + FOLLOW_SLACK_FRAC * abs(cmd_travel):
                        raise MoveError(f"encoder not following: commanded {cmd_travel:+.2f} deg, "
                                        f"measured {p - start:+.2f} deg")
                    if last_p is None or abs(p - last_p) > 0.04:     # ~half a count: the reading moved
                        last_p, cmd_at_change = p, cmd_travel
                    elif abs(cmd_travel - cmd_at_change) > LASH_DEG[self.axis] + STUCK_SLACK_DEG:
                        raise MoveError(f"encoder reading stuck at {p:.2f} deg while commanded "
                                        f"{cmd_travel - cmd_at_change:+.2f} deg")
                    err = target - p
                    if abs(err) < 0.3:               # near the target: decide on a longer average (encoder noise ~1 count)
                        pa = self._pos(n=15)
                        err = target - (p if pa is None else pa)
                    if abs(err) <= self.tol or err * deg < 0:
                        break                # there, or just past it
                    dps = min(vmax, math.sqrt(2.0 * self.accel * abs(err)), self.kp * abs(err))
                    want = tu.out_dps_to_vactual(self.axis, math.copysign(max(self.min_dps, dps), err))
                if now - last_health >= HEALTH_EVERY_S:
                    last_health = now
                    try:
                        self.bus.health(self.addr, self.axis)
                        health_fail_since = None
                    except IOError:
                        health_fail_since = health_fail_since or now
                        if now - health_fail_since > 0.9:
                            raise MoveError(f"{self.axis} driver stopped answering on the UART")
                step = acc_v * dt
                x = int(round(self._cur + max(-step, min(step, want - self._cur))))
                if x != self._cur or now - last_send >= RESEND_S:
                    self._send(x)
                    last_send = now
                time.sleep(LOOP_DT)
            # decelerate to a stop at the same acceleration
            while self._cur != 0:
                if self._estop:
                    raise MoveError("stopped by STOP command")
                step = acc_v * 0.01
                self._send(self._cur - max(-step, min(step, self._cur)))
                time.sleep(0.01)
            time.sleep(0.1)
            p = self._pos(n=15)
            self.last_miss_deg = None if p is None else round(target - p, 3)
        except Exception:
            self.bus.zero_now([self.addr])   # SAFETY: unverified stop first, no reads
            self._cur = 0
            raise
        finally:
            try:
                self.bus.stop_verified(self.addr, release=not self.hold_after_move)
            finally:
                self._cur = 0
                self.is_moving = False

    def stop_movement(self, timeout=1.0):
        """Stop the current move and wait (up to timeout s) for it to finish."""
        self._gen += 1
        if not self.is_moving:
            return True
        deadline = time.monotonic() + timeout
        while self.is_moving and time.monotonic() < deadline:
            time.sleep(0.005)
        if self.is_moving:
            print(f"TMC2209 {self.axis}: move did not stop within {timeout}s; forcing VACTUAL=0")
            self.bus.zero_now([self.addr])
            return False
        return True

    def request_estop(self, send=True):
        """Cut this axis now: flag the move to abort (no ramp) and, if send,
        VACTUAL=0 + current 0 unverified. Doesn't wait. With send=False the
        caller zeroes several axes in one burst (bus.zero_now())."""
        self._estop = True
        self._gen += 1
        if send:
            self.bus.zero_now([self.addr])

    def emergency_stop(self, timeout=1.0, requested=False):
        """STOP button: request_estop() (unless already requested), wait for the
        move to end, then confirm VACTUAL=0 and current 0. True if confirmed."""
        if not requested:
            self.request_estop()
        deadline = time.monotonic() + timeout
        while self.is_moving and time.monotonic() < deadline:
            time.sleep(0.005)
        try:
            self.bus.stop_verified(self.addr, release=True)
            return not self.is_moving
        except Exception as e:
            print(f"TMC2209 {self.axis}: {e}")
            return False

    # ---- speed / ramp configuration (API kept) -----------------------------
    def enable_ramping(self):
        self.ramping_enabled = True

    def disable_ramping(self):
        # Closed-loop moves always ramp; kept so callers don't break.
        self.ramping_enabled = False

    def set_max_speed(self, max_speed_delay):
        if max_speed_delay <= 0:
            raise ValueError("Max speed delay must be greater than 0")
        self.base_delay = max_speed_delay
        self.max_dps = self._delay_to_dps(max_speed_delay)
        print(f"TMC2209 {self.axis}: max speed {self.max_dps:.2f} deg/s (delay {max_speed_delay})")

    def set_speed_range(self, min_delay=None, max_delay=None):
        if min_delay is not None:
            self.set_max_speed(min_delay)

    def set_ramp_profile(self, accel_percent=None, max_accel_steps=None):
        if accel_percent is not None:
            if not 0.0 <= accel_percent <= 0.5:
                raise ValueError("accel_percent must be between 0.0 and 0.5")
            self.ramp_settings['accel_percent'] = accel_percent
        if max_accel_steps is not None:
            if max_accel_steps <= 0:
                raise ValueError("max_accel_steps must be greater than 0")
            self.ramp_settings['max_accel_steps'] = int(max_accel_steps)
            self.accel = min(40.0, max(5.0, 20.0 * 50.0 / max_accel_steps))

    def set_acceleration_rate(self, acceleration_multiplier):
        m = abs(float(acceleration_multiplier) - 1.0)
        if m == 0:
            raise ValueError("Acceleration multiplier must not be 1.0")
        self.set_ramp_profile(max_accel_steps=int(min(400, max(10, 0.5 / m))))

    def set_deceleration_rate(self, deceleration_multiplier):
        pass                                 # symmetric ramp

    def set_gentle_acceleration(self):
        self.accel = 8.0

    def set_moderate_acceleration(self):
        self.accel = 20.0

    def set_aggressive_acceleration(self):
        self.accel = 30.0

    # ---- open-loop step API: not available ---------------------------------
    def step_motor(self, steps, step_delay=None):
        raise NotImplementedError("TMC2209Driver is closed-loop; use step_to()")

    def set_direction(self, clockwise=True):
        raise NotImplementedError("TMC2209Driver is closed-loop; use step_to()")

    @property
    def enable_device(self):
        raise NotImplementedError("TMC2209Driver has no EN pin (EN is tied to GND); "
                                  "backlash_calibration is not supported on this rig")

    def cleanup(self):
        self.stop_movement(timeout=1.0)
        try:
            self.bus.stop_verified(self.addr, release=True)
        except Exception as e:
            print(f"TMC2209 {self.axis}: cleanup: {e}")
