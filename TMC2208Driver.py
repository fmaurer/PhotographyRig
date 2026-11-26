"""STEP/DIR driver for a TMC2208 (standalone mode, no UART).

2026-11-25 rewrite. The version on the rig's SD card was lost; this keeps the
public API websocket_server.py and backlash_calibration.py use, and adds what
they expected but the committed copy lacked (total_signed_steps,
reset_step_counter, _direction). Changes from the committed version:

  * Step counter: total_signed_steps counts every pulse actually sent (also
    when a move is cancelled, and for backlash_calibration's direct calls).
  * Real acceleration ramp. The old multipliers (1.000001 / 0.999999) grew
    the delay instead of shrinking it, so a move ran up to 50 steps at 1/4
    speed, jumped straight to full speed, then dropped straight back to 1/4
    speed: a 4x step change in speed, a common cause of lost steps.
    Now speed follows a constant-acceleration curve
    v(n) = sqrt(v0^2 + (vmax^2 - v0^2) * n / N) up and down.
  * Deadline timing. Each pulse is scheduled against an absolute clock
    and the wait spins on sched_yield() for the last ~1 ms. time.sleep(25e-6)
    really sleeps ~77 us on a Pi 5, so the pan axis ran ~3x slower than its
    setting, and the errors piled up. sched_yield() releases
    the GIL, so pan and tilt threads can both run.
  * stop_movement() gives up after a timeout instead of waiting forever.
  * Optional soft travel limits (in steps from the counter's zero).
  * Optional one-direction final approach (same idea as the TMC2209 rig's
    return-to-start): a move that ends going against approach_dir overshoots
    by approach_overshoot steps and comes back, so every stop is reached from
    the same gear flank and the step->angle map stays consistent. Off by
    default. If you turn it on, turn OFF the browser-side backlash
    compensation in index.html, or the two will fight.
  * step_to_angle() set DIR after stepping; fixed.

Still open-loop. Python can't time step pulses accurately enough. For
smooth, exact speeds generate the pulses outside Python (lgpio tx_pulse,
which runs in a C thread) or wire PDN_UART and use the driver's VACTUAL
register, like the TMC2209 rig does.
"""

import math
import os
import time

from gpiozero import OutputDevice

from motor_driver import MotorDriver

_SPIN_BELOW_S = 0.0012   # sleep normally until this close to a deadline, then spin


def _wait_until(deadline):
    """Wait until perf_counter() >= deadline. Coarse sleep first (it overshoots
    by ~50 us), then spin on sched_yield(), which releases the GIL so the
    other axis' thread keeps running."""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > _SPIN_BELOW_S:
            time.sleep(remaining - _SPIN_BELOW_S)
        else:
            os.sched_yield()   # releases the GIL; time.sleep(0) costs ~50 us of timer slack


class TMC2208Driver(MotorDriver):
    def __init__(self, step_pin, dir_pin, enable_pin, step_delay,
                 soft_limits=None, approach_dir=0, approach_overshoot=0,
                 hold_after_move=False, dir_invert=False):
        """step_delay: seconds for each half of the step pulse (period = 2x),
        same meaning as before. soft_limits: (min_steps, max_steps) or None.
        approach_dir: +1 / -1 / 0 (off). hold_after_move: keep the coils
        powered between moves (EN low) so the gear can't back-drive; the old
        behaviour (False) cuts the current after every move."""
        self.step_device = OutputDevice(step_pin)
        self.dir_device = OutputDevice(dir_pin)
        self.enable_device = OutputDevice(enable_pin, initial_value=True)  # EN high = off
        self.base_delay = step_delay
        self.dir_invert = dir_invert

        # Movement control
        self.should_stop = False
        self.is_moving = False

        # Position
        self.total_signed_steps = 0
        self._direction = 1
        self.soft_limits = tuple(soft_limits) if soft_limits else None
        self.approach_dir = int(approach_dir)
        self.approach_overshoot = int(approach_overshoot)
        self.hold_after_move = hold_after_move

        # Speed range (delays are half-periods, as before)
        self.min_delay = step_delay          # fastest
        self.max_delay = step_delay * 4      # start/stop speed
        # Kept for API compatibility; the ramp is now set by ramp_settings.
        self.acceleration = 1.0
        self.deceleration = 1.0

        self.ramping_enabled = True
        self.ramp_settings = {
            'accel_percent': 0.3,     # fraction of the move used to accelerate (same again to stop)
            'max_accel_steps': 50,    # cap on ramp length in steps
        }

    # ---- position ---------------------------------------------------------
    def reset_step_counter(self, value=0):
        self.total_signed_steps = int(value)

    def set_soft_limits(self, min_steps=None, max_steps=None):
        self.soft_limits = None if min_steps is None and max_steps is None else (
            -math.inf if min_steps is None else min_steps,
            math.inf if max_steps is None else max_steps)

    def set_approach(self, direction=0, overshoot_steps=0):
        """direction +1/-1 enables the one-direction final approach, 0 disables."""
        self.approach_dir = int(direction)
        self.approach_overshoot = int(overshoot_steps)

    def _clamp_to_limits(self, signed_steps):
        if not self.soft_limits:
            return signed_steps
        lo, hi = self.soft_limits
        target = self.total_signed_steps + signed_steps
        clamped = min(max(target, lo), hi)
        if clamped != target:
            print(f"TMC2208: move of {signed_steps} would reach {target}, outside soft limits "
                  f"[{lo}, {hi}]; clamped to {clamped}")
        return int(clamped - self.total_signed_steps)

    # ---- moves ------------------------------------------------------------
    def num_steps_for_angle(self, degrees):
        # NOTE: assumes FULL steps. In standalone mode a TMC2208 defaults to
        # microstepping set by MS1/MS2, so the real steps-per-degree is
        # usually 8x or 16x this; the pointing/IMU calibration measures it
        # instead of trusting this number.
        stepper_step_size = 1.8  # degrees
        driver_gear_teeth = 12
        bevel_gear_teeth = 12
        ring_gear_teeth = 48

        stage_0 = bevel_gear_teeth / driver_gear_teeth
        stage_1 = ring_gear_teeth / bevel_gear_teeth

        reduction = stage_1 / stage_0

        return round(degrees / (stepper_step_size / reduction))

    def step_to_angle(self, degrees):
        self.step_to(self.num_steps_for_angle(degrees))
        return 0

    def step_to(self, number):
        """Relative move of `number` signed steps (the name is historical)."""
        number = self._clamp_to_limits(int(number))
        if number == 0:
            return
        a, o = self.approach_dir, self.approach_overshoot
        if a and o > 0 and (number > 0) != (a > 0):
            # Ends moving against the approach direction: overshoot, then come back.
            over = self._clamp_to_limits(number - a * o)
            segments = [over, number - over]
        else:
            segments = [number]
        self._begin()
        try:
            for seg in segments:
                if self.should_stop:
                    break
                if seg:
                    self._set_dir(1 if seg > 0 else -1)
                    self._segment(abs(seg))
        finally:
            self._end()

    def _set_dir(self, sign):
        self._direction = 1 if sign > 0 else -1
        self.dir_device.value = (self._direction > 0) != self.dir_invert

    def stop_movement(self, timeout=1.0):
        """Ask the current move to stop and wait (up to timeout s) for it."""
        if not self.is_moving:
            return True
        self.should_stop = True
        deadline = time.perf_counter() + timeout
        while self.is_moving and time.perf_counter() < deadline:
            time.sleep(0.001)
        stopped = not self.is_moving
        if not stopped:
            print(f"TMC2208: move did not stop within {timeout}s")
        self.should_stop = False
        return stopped

    # ---- ramp configuration (API unchanged) -------------------------------
    def enable_ramping(self):
        self.ramping_enabled = True
        print("Acceleration ramping enabled")

    def disable_ramping(self):
        self.ramping_enabled = False
        print("Acceleration ramping disabled")

    def set_ramp_profile(self, accel_percent=None, max_accel_steps=None):
        if accel_percent is not None:
            if 0.0 <= accel_percent <= 0.5:
                self.ramp_settings['accel_percent'] = accel_percent
            else:
                raise ValueError("accel_percent must be between 0.0 and 0.5")
        if max_accel_steps is not None:
            if max_accel_steps > 0:
                self.ramp_settings['max_accel_steps'] = int(max_accel_steps)
            else:
                raise ValueError("max_accel_steps must be greater than 0")

    def set_acceleration_rate(self, acceleration_multiplier):
        """Kept for the UI's custom-ramp control. The multiplier no longer
        shapes the curve directly; it scales the ramp length instead:
        closer to 1.0 -> longer, gentler ramp."""
        m = abs(float(acceleration_multiplier) - 1.0)
        if m == 0:
            raise ValueError("Acceleration multiplier must not be 1.0")
        self.acceleration = float(acceleration_multiplier)
        self.ramp_settings['max_accel_steps'] = int(min(400, max(10, 0.5 / m)))
        print(f"Acceleration ramp length set to {self.ramp_settings['max_accel_steps']} steps")

    def set_deceleration_rate(self, deceleration_multiplier):
        # The ramp is symmetric; deceleration uses the same length.
        self.deceleration = float(deceleration_multiplier)

    def set_speed_range(self, min_delay=None, max_delay=None):
        if min_delay is not None:
            self.min_delay = min_delay
        if max_delay is not None:
            self.max_delay = max_delay
        print(f"Speed range: min_delay={self.min_delay}, max_delay={self.max_delay}")

    def set_max_speed(self, max_speed_delay):
        if max_speed_delay <= 0:
            raise ValueError("Max speed delay must be greater than 0")
        self.min_delay = max_speed_delay
        print(f"Max speed set to: {max_speed_delay} seconds delay (faster = smaller delay)")

    def set_gentle_acceleration(self):
        self.ramp_settings['accel_percent'] = 0.4
        self.ramp_settings['max_accel_steps'] = 200
        print("Gentle acceleration profile enabled")

    def set_moderate_acceleration(self):
        self.ramp_settings['accel_percent'] = 0.3
        self.ramp_settings['max_accel_steps'] = 80
        print("Moderate acceleration profile enabled")

    def set_aggressive_acceleration(self):
        self.ramp_settings['accel_percent'] = 0.2
        self.ramp_settings['max_accel_steps'] = 25
        print("Aggressive acceleration profile enabled")

    # ---- stepping ---------------------------------------------------------
    def _release(self):
        if not self.hold_after_move:
            self.enable_device.value = True

    def _begin(self):
        self.should_stop = False
        self.is_moving = True
        self.enable_device.value = False

    def _end(self):
        self.is_moving = False
        self._release()

    def _segment(self, steps):
        if self.ramping_enabled:
            self._step_motor_with_ramping(steps)
        else:
            self._step_motor_constant_speed(steps)

    def step_motor(self, steps, step_delay=None):
        """Old API: `steps` in the direction already set on the DIR pin."""
        if steps <= 0:
            return
        if step_delay is not None:
            self.base_delay = step_delay
        self._begin()
        try:
            self._segment(steps)
        finally:
            self._end()

    def _ramp_len(self, steps):
        return min(int(steps * self.ramp_settings['accel_percent']),
                   self.ramp_settings['max_accel_steps'])

    def _period_at(self, i, steps, n_ramp):
        """Full step period for step i of `steps` (constant-acceleration ramp)."""
        v_max = 1.0 / (2.0 * self.min_delay)
        v0 = min(1.0 / (2.0 * self.max_delay), v_max)
        if n_ramp <= 0:
            return 1.0 / v_max
        k = min(i + 1, steps - i, n_ramp) / n_ramp       # 0..1 up, 1 cruise, ..0 down
        v = math.sqrt(v0 * v0 + (v_max * v_max - v0 * v0) * min(k, 1.0))
        return 1.0 / v

    def _run_steps(self, steps, period_fn):
        t = time.perf_counter()
        for i in range(steps):
            if self.should_stop:
                print("Movement stopped by stop command")
                return
            period = period_fn(i)
            self.step_device.value = True
            self.total_signed_steps += self._direction
            _wait_until(t + period * 0.5)
            self.step_device.value = False
            # Next edge one period after this one, but never earlier than now:
            # if we were held up (GIL, GC, another process), carry on at the
            # set speed instead of sending a burst of fast steps to catch up.
            t = max(t + period, time.perf_counter())
            _wait_until(t)

    def _step_motor_constant_speed(self, steps):
        self._run_steps(steps, lambda i: 2.0 * self.base_delay)

    def _step_motor_with_ramping(self, steps):
        n_ramp = self._ramp_len(steps)
        self._run_steps(steps, lambda i: self._period_at(i, steps, n_ramp))

    def _do_step(self, delay):
        """Single step (kept for callers of the old API)."""
        self.step_device.value = True
        self.total_signed_steps += self._direction
        _wait_until(time.perf_counter() + delay)
        self.step_device.value = False
        _wait_until(time.perf_counter() + delay)

    def set_direction(self, clockwise=True):
        self._set_dir(1 if clockwise else -1)

    def cleanup(self):
        self.step_device.off()
        self.dir_device.off()
        self.enable_device.on()   # EN high = driver off
