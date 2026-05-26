"""Backlash calibration via optical feedback on the MediaMTX RTSP stream.

Measures the gear backlash (in motor steps) for the yaw and pitch axes in
both directions. For each (axis, direction) we drive a large move to load
one gear flank, then send increasing reverse-step counts and watch for the
first commanded count that produces detectable image motion. The smallest
N that breaks past the flank is the backlash, in steps.

Triggered from websocket_server.py. Runs inside the main rig process
because the main rig already owns the motor controllers and the cameras.
Frames are consumed from rtsp://localhost:8554/cam{N} (MediaMTX); if the
stream isn't available we abort without falling back to direct Picamera2.

Compensation is OUT OF SCOPE — this module measures only and writes a
JSON next to the other calibration artifacts.
"""

import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable, Optional

import cv2
import numpy as np

# matplotlib is optional: the calibration must succeed even on a Pi that
# never installed it. When unavailable we skip plot rendering and the
# WebSocket result simply omits `plot_url`.
try:
    import matplotlib
    matplotlib.use("Agg")  # headless: no Pi display required
    import matplotlib.pyplot as _plt
    _MATPLOTLIB_AVAILABLE = True
except ImportError:
    _plt = None
    _MATPLOTLIB_AVAILABLE = False


# Geometry constants ---------------------------------------------------------

# Output (camera) rotation per motor step, derived from TMC2208Driver
# num_steps_for_angle (lines 35-46): 1.8 deg/step / reduction 4 = 0.45 deg/step.
DEG_PER_STEP = 1.8 / 4.0

# Default horizontal FOV for cam0 (zoom). Used to compute deg_per_pixel for
# the FOV-based max_steps cap. The actual lens FOV may differ; override via
# the `cam_horizontal_fov_deg` kwarg if you've measured it.
DEFAULT_CAM0_HFOV_DEG = 30.0
DEFAULT_CAM1_HFOV_DEG = 60.0


@dataclass
class _TrialFit:
    """One trial's piecewise-fit result for a single (axis, direction).

    `k_deadband` is how many of the leading probes the algorithm decided
    were inside the backlash deadband (shift ≈ 0). K=0 means a clean
    linear regime through all probes; K>=1 means the first K probes were
    excluded from the linear fit because they sit inside the deadband.

    `fit_data` is the raw list of (N_steps, shift_mag_px, dx, dy) the
    trial actually observed — preserved so a notebook can re-fit or you
    can spot a single bad probe pulling the fit off.
    """
    backlash_steps: float
    slope_px_per_step: float
    r_squared: float
    intercept_px: float
    k_deadband: int = 0
    fit_data: list = field(default_factory=list)


@dataclass
class _AxisResult:
    trials: list = field(default_factory=list)  # list[_TrialFit]


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _theil_sen_fit(ns: np.ndarray, mags: np.ndarray) -> tuple:
    """Robust linear regression. Slope = median of all pairwise slopes;
    intercept = median(mag - slope * n). Tolerates ~29% outliers without
    any threshold tuning — exactly what we need when the smallest probes
    are noise-dominated (below backlash) and the largest may be saturated
    (scene leaving frame).

    With 5 probes this is C(5,2)=10 pairwise slopes — deterministic,
    sub-millisecond, no randomness.
    """
    slopes = []
    n = len(ns)
    for i in range(n):
        for j in range(i + 1, n):
            dn = ns[j] - ns[i]
            if dn != 0:
                slopes.append((mags[j] - mags[i]) / dn)
    if not slopes:
        return 0.0, 0.0
    slope = float(np.median(slopes))
    intercept = float(np.median(mags - slope * ns))
    return slope, intercept


_DEADBAND_NOISE_THRESHOLD_PX = 2.0


def _piecewise_fit(ns: np.ndarray, mags: np.ndarray,
                   noise_threshold_px: float = _DEADBAND_NOISE_THRESHOLD_PX
                   ) -> tuple:
    """Piecewise deadband+linear fit. Probes whose shift magnitude is
    below `noise_threshold_px` AND form a contiguous leading run are
    classified as the deadband (shift = 0); the rest are fit linearly.

    Returns (slope, intercept, k_deadband, r_squared):
      - K = 0 means no deadband — slope/intercept from Theil-Sen on all
        probes for outlier robustness.
      - K >= 1 means the first K probes were below the noise threshold;
        slope/intercept from least-squares on the remaining N-K probes.

    The threshold rule beats SSE minimisation here because pure SSE will
    happily classify a probe with 3 px of real motion as deadband if it
    lowers the linear segment's SSE — under-extending the linear fit
    and inflating the backlash. The threshold approach respects the
    actual measurement: if a probe shifted more than the noise floor,
    the gear has engaged and the probe belongs in the linear segment.

    Falls back to all-linear Theil-Sen if fewer than 2 probes are above
    threshold (so the linear fit has too few points).
    """
    n = len(ns)
    if n < 2:
        return 0.0, 0.0, 0, 0.0
    # Find the leading run of below-threshold probes — that's the
    # deadband. Once we hit a probe with real motion, everything from
    # there on counts as the linear regime, even if a later probe happens
    # to dip back below threshold (which would be a tracking blip, not a
    # return to the deadband).
    k = 0
    for i in range(n):
        if mags[i] < noise_threshold_px:
            k = i + 1
        else:
            break
    lin_ns = ns[k:]
    lin_mags = mags[k:]
    if len(lin_ns) < 2:
        # Either the data is all-noise or it spikes only on the last
        # probe — either way the threshold rule can't fit. Fall back to
        # Theil-Sen on everything; the slope sanity check upstream
        # handles the noise-only case.
        slope, intercept = _theil_sen_fit(ns, mags)
        k = 0
    else:
        slope_arr = np.polyfit(lin_ns, lin_mags, 1)
        slope = float(slope_arr[0])
        intercept = float(slope_arr[1])
    # R² against the chosen piecewise model — predicted = 0 for the
    # deadband indices, slope*n + intercept for the linear indices.
    idx = np.arange(n)
    predictions = np.where(idx < k, 0.0, slope * ns + intercept)
    ss_res = float(np.sum((mags - predictions) ** 2))
    ss_tot = float(np.sum((mags - float(np.mean(mags))) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0
    return slope, intercept, k, r_squared


def _step_holding_enable(driver, signed_steps: int) -> None:
    """Drive a relative move WITHOUT releasing the driver enable pin.

    Bypasses TMC2208Driver.step_motor whose `finally` clause sets
    enable_device.value = True (coils unpowered) on every call — between
    probes the gearbox would be free to back-drive under any preload,
    corrupting sub-step backlash readings. The outer run() holds enable low
    for the entire calibration and re-disables on exit.
    """
    if signed_steps == 0:
        return
    driver.dir_device.value = signed_steps > 0
    driver._direction = 1 if signed_steps > 0 else -1
    driver.is_moving = True
    driver.should_stop = False
    try:
        n = abs(signed_steps)
        if driver.ramping_enabled:
            driver._step_motor_with_ramping(n)
        else:
            driver._step_motor_constant_speed(n)
    finally:
        driver.is_moving = False
        # Intentionally NOT touching driver.enable_device here.


def _render_backlash_plot(payload: dict, output_path: str) -> bool:
    """Render a 2x2 matplotlib PNG summarising one calibration run. Returns
    True on success, False if matplotlib is unavailable or rendering
    failed. Never raises — the caller treats the file as best-effort.

    The plot has one subplot per (axis, direction) showing every trial's
    raw (N, shift) probes, the mean piecewise fit line (flat at 0 in the
    deadband, sloped past it), and a vertical dashed marker at the
    measured backlash. Title text is red when the result is flagged
    `low_confidence`.
    """
    if not _MATPLOTLIB_AVAILABLE:
        print("backlash_calibration: matplotlib not available; skipping plot")
        return False
    try:
        results = payload.get("results", {}) or {}
        axes_order = [("yaw", "pos"), ("yaw", "neg"),
                      ("pitch", "pos"), ("pitch", "neg")]
        fig, axarr = _plt.subplots(2, 2, figsize=(11, 8))
        axarr_flat = axarr.flat
        for ax_plot, (axis, direction) in zip(axarr_flat, axes_order):
            key = f"{axis}_{direction}"
            entry = results.get(key, {}) or {}
            bl_block = entry.get("backlash_steps", {}) or {}
            sl_block = entry.get("slope_px_per_step", {}) or {}
            backlash_mean = bl_block.get("mean")
            backlash_std = bl_block.get("std", 0.0)
            slope_mean = sl_block.get("mean", 0.0)
            r2_mean = entry.get("r_squared_mean")
            low_conf = bool(entry.get("low_confidence", False))
            near_zero = bool(entry.get("near_zero_backlash", False))
            fit_data_trials = entry.get("fit_data", []) or []

            # Scatter every trial's (N, mag) probes, color per trial.
            all_ns = []
            for ti, trial in enumerate(fit_data_trials):
                if not trial:
                    continue
                ns = [p[0] for p in trial]
                mags = [p[1] for p in trial]
                all_ns.extend(ns)
                ax_plot.scatter(ns, mags, s=28, alpha=0.8,
                                label=f"trial {ti+1}")

            # Mean piecewise fit line: 0 in the deadband, slope past it.
            if all_ns and backlash_mean is not None and slope_mean:
                x_max = float(max(all_ns)) * 1.05
                # Intercept implied by the fit: 0 = slope * backlash + b
                #   → b = -slope * backlash.
                b = -float(slope_mean) * float(backlash_mean)
                x_dead = np.linspace(0.0, max(0.0, float(backlash_mean)), 2)
                ax_plot.plot(x_dead, np.zeros_like(x_dead),
                             color="#888", linewidth=1.5, linestyle="-",
                             label="fit (deadband)")
                if x_max > float(backlash_mean):
                    x_lin = np.linspace(float(backlash_mean), x_max, 2)
                    y_lin = float(slope_mean) * x_lin + b
                    ax_plot.plot(x_lin, y_lin, color="#FF9800",
                                 linewidth=1.8, label="fit (linear)")
                ax_plot.axvline(x=float(backlash_mean), color="#FF5252",
                                linestyle="--", linewidth=1.2,
                                label=f"B={float(backlash_mean):.0f}")

            ax_plot.set_xlabel("probe N (steps)")
            ax_plot.set_ylabel("shift magnitude (px)")
            ax_plot.grid(True, alpha=0.3)
            title_parts = [key]
            if backlash_mean is not None:
                title_parts.append(
                    f"B={backlash_mean:.0f}±{backlash_std:.0f}")
            if slope_mean:
                title_parts.append(f"slope={slope_mean:.3f}px/step")
            if r2_mean is not None:
                title_parts.append(f"R²={r2_mean:.2f}")
            title = "  ".join(title_parts)
            # Three-state title colour:
            #   red   — fit quality is bad (R² low or trial spread big on
            #           a real deadband); trust nothing.
            #   grey  — fit is clean but backlash ≈ 0; no compensation
            #           needed, algorithm isn't worried.
            #   black — normal: measurable deadband, trustworthy.
            if low_conf:
                title_color = "#C62828"
            elif near_zero:
                title_color = "#666666"
            else:
                title_color = "black"
            ax_plot.set_title(title, color=title_color, fontsize=10)
            ax_plot.legend(loc="upper left", fontsize=7)

        cam_idx = payload.get("camera_idx", "?")
        when = payload.get("calibrated_at", "")
        fig.suptitle(f"Backlash calibration — cam{cam_idx}  {when}",
                     fontsize=12)
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
        fig.savefig(output_path, dpi=100, bbox_inches="tight")
        _plt.close(fig)
        return True
    except Exception as e:
        print(f"backlash_calibration: plot render failed: {e!r}")
        try:
            _plt.close("all")
        except Exception:
            pass
        return False


class _RtspReader:
    """Continuously reads frames from an RTSP stream into a 'latest frame'
    slot. Avoids the stale-RTSP-frame problem you hit when you sleep for
    longer than the FFmpeg buffer and then cap.read() returns an old (or
    eventually no) frame — the consumer (us) falls behind, libavformat
    back-pressures, and the read pipeline stalls or drops.

    The reader thread keeps draining the stream at full rate; callers get
    whatever's most recent via latest(). Worst case staleness is ~one
    frame interval (~33ms at 30fps).
    """

    def __init__(self, url):
        self.url = url
        self._cap = None
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def open(self) -> bool:
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            time.sleep(0.5)
            cap.release()
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            cap.release()
            return False
        # Hint the backend to keep just one frame buffered. Often ignored
        # on FFmpeg but harmless.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        # Prime with a real frame so frame_size is known before the
        # consumer asks for it.
        ok, frame = cap.read()
        retries = 10
        while (not ok or frame is None) and retries > 0:
            time.sleep(0.2)
            ok, frame = cap.read()
            retries -= 1
        if not ok or frame is None:
            cap.release()
            return False
        self._cap = cap
        with self._lock:
            self._latest = frame
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="rtsp-reader")
        self._thread.start()
        return True

    def _run(self):
        while not self._stop.is_set():
            try:
                ok, frame = self._cap.read()
            except Exception:
                time.sleep(0.1)
                continue
            if ok and frame is not None:
                with self._lock:
                    self._latest = frame
            else:
                time.sleep(0.05)

    def latest(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    @property
    def frame_size(self):
        with self._lock:
            if self._latest is None:
                return None
            h, w = self._latest.shape[:2]
            return (int(w), int(h))

    def stop(self):
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None


class CalibrationCancelled(Exception):
    pass


class CalibrationAborted(Exception):
    """Raised for non-cancel terminating conditions (low texture, RTSP gone…)."""
    def __init__(self, reason: str, detail: str = ""):
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


class BacklashCalibrator:
    """One-shot backlash measurement runner. Call run() from a worker thread.

    The run interleaves (yaw_pos, pitch_pos, yaw_neg, pitch_neg) trials so
    thermal/mechanical drift is spread across all four combinations rather
    than concentrated in whichever one happens to run first.
    """

    AXES = ("yaw", "pitch")
    DIRECTIONS = ("pos", "neg")

    def __init__(self,
                 pan_motor_controller,
                 tilt_motor_controller,
                 camera_manager,
                 cam_idx: int,
                 *,
                 trials: int = 5,
                 max_steps: int = 512,
                 settle_s: float = 3.0,
                 engage_steps: int = 150,
                 min_features: int = 20,
                 probe_steps: Optional[list] = None,
                 pitch_probe_steps: Optional[list] = None,
                 cam_horizontal_fov_deg: Optional[float] = None,
                 rtsp_host: str = "localhost",
                 output_path: Optional[str] = None,
                 debug_dir: Optional[str] = None,
                 progress_callback: Optional[Callable[[dict], None]] = None):
        self.pan_mc = pan_motor_controller
        self.tilt_mc = tilt_motor_controller
        self.camera_manager = camera_manager
        self.cam_idx = int(cam_idx)
        self.trials = int(trials)
        self.max_steps_user = int(max_steps)
        self.settle_s = float(settle_s)
        self.engage_steps = int(engage_steps)
        self.min_features = int(min_features)
        # Default sweep — sized so even the loosely-geared (yaw on this
        # rig) axis clears its backlash. The FOV cap in
        # _init_window_and_cap_steps is now informational only — probes
        # pass through verbatim. KLT failures (probe moved scene out of
        # frame, no features tracked) are skipped from the fit at runtime.
        # Yaw and pitch typically have wildly different gear ratios. The
        # probe_steps list is the default (used for both axes); set
        # pitch_probe_steps to override on the pitch axis only. Same
        # backward-compat semantics as before when pitch_probe_steps is
        # None.
        # Default sweep tries to cover both:
        #   - yaw_neg-style small (~30-50 step) backlashes (probes at 50,100)
        #   - yaw_pos-style large (~200+) backlashes (probes at 300,400)
        # Top probe of 400 keeps motion comfortably inside KLT's effective
        # search range so we don't pollute the fit with bad tracks.
        self.probe_steps = (list(probe_steps) if probe_steps
                            else [50, 100, 200, 300, 400])
        self.pitch_probe_steps = (list(pitch_probe_steps)
                                  if pitch_probe_steps else None)
        self._effective_probe_steps: list = []        # yaw / fallback
        self._effective_pitch_probe_steps: list = []  # pitch override
        # KLT tuning. Reasonable for 720p+ frames; lower minDistance for
        # higher-res ones if features cluster.
        self._gftt_params = dict(maxCorners=300, qualityLevel=0.01,
                                 minDistance=8, blockSize=7)
        # Bigger window + extra pyramid level = trackable motion ~2x larger.
        # With winSize=31 and maxLevel=5, KLT can resolve displacements up
        # to roughly 31 * 2^5 / 2 ≈ 500 px (vs ~84 px at winSize=21,
        # maxLevel=3). Doesn't help when scene leaves frame, but does
        # rescue probes that are simply large.
        self._lk_params = dict(winSize=(31, 31), maxLevel=5,
                               criteria=(cv2.TERM_CRITERIA_EPS
                                         | cv2.TERM_CRITERIA_COUNT,
                                         30, 0.01))
        # Most recent successfully-tracked (src, dst) pairs, set by
        # _estimate_shift; used by the debug composite renderer.
        self._last_tracks = None
        if cam_horizontal_fov_deg is None:
            cam_horizontal_fov_deg = (DEFAULT_CAM0_HFOV_DEG if self.cam_idx == 0
                                      else DEFAULT_CAM1_HFOV_DEG)
        self.cam_hfov_deg = float(cam_horizontal_fov_deg)
        self.rtsp_host = rtsp_host
        self.output_path = output_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "backlash_calibration.json")
        self.debug_dir = debug_dir
        self._progress_cb = progress_callback or (lambda m: None)

        self.cancel_event = threading.Event()
        self._debug_rows: list = []

        # Filled during run().
        self._reader: Optional[_RtspReader] = None
        self._frame_size: Optional[tuple] = None
        self._max_steps_effective: int = self.max_steps_user

    # ----- public --------------------------------------------------------

    @property
    def rtsp_url(self) -> str:
        return f"rtsp://{self.rtsp_host}:8554/cam{self.cam_idx}"

    def run(self) -> dict:
        """Execute the full calibration. Returns the result dict that will
        be persisted to JSON. Raises CalibrationCancelled if cancelled, or
        CalibrationAborted for hard failures (rtsp_unavailable, low_texture).
        """
        self._progress({"type": "backlash_progress", "phase": "starting",
                        "rtsp_url": self.rtsp_url})

        ae_was_locked = self._lock_ae(True)
        ramping_state = {
            "pan": self.pan_mc.driver.ramping_enabled,
            "tilt": self.tilt_mc.driver.ramping_enabled,
        }
        # Hold both drivers energised for the duration.
        self.pan_mc.driver.enable_device.value = False
        self.tilt_mc.driver.enable_device.value = False

        if self.debug_dir:
            os.makedirs(self.debug_dir, exist_ok=True)
            self._progress({"type": "backlash_progress", "phase": "debug_dir",
                            "path": self.debug_dir})

        # One-shot initial settle. The rig may still be moving from whatever
        # the user just did (clicking the calibrate button on a desk-mounted
        # rig, dropping their phone on the bench, etc). Let it stop ringing
        # before we capture our first reference frame.
        self._progress({"type": "backlash_progress", "phase": "initial_settle",
                        "duration_s": self.settle_s})
        time.sleep(self.settle_s)

        # Snap one frame *before* any motion so we always have evidence in
        # the debug dir, even if a later texture_check abort fires before
        # the noise-floor loop saves any composites. Lets the user verify
        # the RTSP frame is real (not all-black) and that focus is OK.
        if self.debug_dir:
            try:
                initial = self._capture_fresh()
                self._save_raw_debug_frame(initial, "initial_capture.png")
                lap_var = float(cv2.Laplacian(
                    initial,
                    cv2.CV_32F if initial.dtype == np.float32 else cv2.CV_64F
                ).var())
                self._progress({
                    "type": "backlash_progress", "phase": "initial_capture",
                    "laplacian_var": lap_var,
                    "frame_size": list(self._frame_size),
                })
            except Exception as e:
                print(f"backlash_calibration: initial debug capture failed: {e!r}")

        try:
            self._open_rtsp()
            self._init_window_and_cap_steps()

            results: dict = {f"{ax}_{d}": _AxisResult()
                             for ax in self.AXES for d in self.DIRECTIONS}
            warnings: list = []
            # Track which (axis, direction) combos have passed the texture
            # check so we don't redo it on every trial.
            texture_ok: set = set()

            # Interleaved trial order: (yaw_pos, pitch_pos, yaw_neg, pitch_neg) × trials
            for trial_idx in range(1, self.trials + 1):
                for direction in self.DIRECTIONS:
                    for axis in self.AXES:
                        if self.cancel_event.is_set():
                            raise CalibrationCancelled()
                        key = f"{axis}_{direction}"
                        do_tex_check = key not in texture_ok
                        trial_fit = self._measure_one_trial_fit(
                            axis, direction, trial_idx,
                            do_texture_check=do_tex_check)
                        if do_tex_check:
                            texture_ok.add(key)
                        results[key].trials.append(trial_fit)
                        self._progress({
                            "type": "backlash_trial_done",
                            "axis": axis, "direction": direction,
                            "trial": trial_idx,
                            "backlash_steps": trial_fit.backlash_steps,
                            "slope_px_per_step": trial_fit.slope_px_per_step,
                            "r_squared": trial_fit.r_squared,
                        })

            summary = self._aggregate(results, warnings)
            payload = {
                "calibrated_at": _utcnow_iso(),
                "camera_idx": self.cam_idx,
                "rtsp_url": self.rtsp_url,
                "frame_size": list(self._frame_size),
                "settle_s": self.settle_s,
                "engage_steps": self.engage_steps,
                "max_steps_effective": self._max_steps_effective,
                "min_features": self.min_features,
                "probe_steps_user": list(self.probe_steps),
                "probe_steps_effective": list(self._effective_probe_steps),
                "cam_horizontal_fov_deg": self.cam_hfov_deg,
                "deg_per_step": DEG_PER_STEP,
                "trials_requested": self.trials,
                "results": summary,
                "warnings": warnings,
            }
            self._write_json(payload)
            # Render a 2x2 matplotlib summary alongside the JSON. Saved
            # both at the stable project-root path and (when debug is
            # on) inside the timestamped debug directory. Best-effort:
            # rendering failure does not fail the calibration.
            stable_plot_path = os.path.join(
                os.path.dirname(os.path.abspath(self.output_path)),
                "backlash_plot.png")
            plot_ok = _render_backlash_plot(payload, stable_plot_path)
            payload["plot_path"] = stable_plot_path if plot_ok else None
            if self.debug_dir and plot_ok:
                try:
                    debug_plot_path = os.path.join(self.debug_dir,
                                                   "backlash_plot.png")
                    _render_backlash_plot(payload, debug_plot_path)
                    payload["debug_plot_path"] = debug_plot_path
                except Exception as e:
                    print(f"backlash_calibration: debug plot render "
                          f"failed: {e!r}")
            self._progress({"type": "backlash_progress", "phase": "done",
                            "path": self.output_path,
                            "plot_path": payload.get("plot_path")})
            return payload

        finally:
            # Restore everything we touched.
            try:
                self.pan_mc.driver.ramping_enabled = ramping_state["pan"]
            except Exception:
                pass
            try:
                self.tilt_mc.driver.ramping_enabled = ramping_state["tilt"]
            except Exception:
                pass
            try:
                self.pan_mc.driver.enable_device.value = True
                self.tilt_mc.driver.enable_device.value = True
            except Exception:
                pass
            if ae_was_locked:
                self._lock_ae(False)
            if self._reader is not None:
                try:
                    self._reader.stop()
                except Exception:
                    pass
                self._reader = None
            # Always flush the debug log so partial runs (cancel, abort) are
            # still inspectable.
            if self.debug_dir and self._debug_rows:
                try:
                    with open(os.path.join(self.debug_dir, "summary.json"), "w") as f:
                        json.dump(self._debug_rows, f, indent=2)
                except Exception as e:
                    print(f"backlash_calibration: failed to write summary.json: {e!r}")

    # ----- setup ---------------------------------------------------------

    def _open_rtsp(self) -> None:
        url = self.rtsp_url
        reader = _RtspReader(url)
        if not reader.open():
            raise CalibrationAborted("rtsp_unavailable", url)
        self._reader = reader
        self._frame_size = reader.frame_size
        if self._frame_size is None:
            raise CalibrationAborted("rtsp_no_frames", url)

    def _init_window_and_cap_steps(self) -> None:
        w, h = self._frame_size
        # The FOV cap based on the DEG_PER_STEP constant assumed a 4:1 gear
        # reduction, but real rigs have wildly different per-axis gear
        # ratios (this rig: yaw ~2.67x more geared down than pitch). The
        # cap was too aggressive. Now we pass user-supplied probe sizes
        # through verbatim — if a probe is too big and the camera leaves
        # the frame, KLT will return zero features and the resulting (0,0)
        # shift is filtered from the fit at runtime.
        deg_per_pixel = self.cam_hfov_deg / float(w)
        self._max_steps_effective = int(self.max_steps_user)
        effective = sorted({int(p) for p in self.probe_steps if int(p) > 0})
        self._effective_probe_steps = effective
        if self.pitch_probe_steps:
            self._effective_pitch_probe_steps = sorted(
                {int(p) for p in self.pitch_probe_steps if int(p) > 0})
        else:
            self._effective_pitch_probe_steps = []  # pitch uses default
        self._progress({
            "type": "backlash_progress", "phase": "init",
            "frame_size": list(self._frame_size),
            "deg_per_step_nominal": DEG_PER_STEP,
            "deg_per_pixel_nominal": deg_per_pixel,
            "max_steps_user": self.max_steps_user,
            "max_steps_effective": self._max_steps_effective,
            "probe_steps_user": list(self.probe_steps),
            "probe_steps_effective": list(self._effective_probe_steps),
            "pitch_probe_steps_user": (list(self.pitch_probe_steps)
                                        if self.pitch_probe_steps else None),
            "pitch_probe_steps_effective": list(self._effective_pitch_probe_steps),
        })

    def _lock_ae(self, lock: bool) -> bool:
        """Try to disable autoexposure on the chosen camera. Returns True if
        the call succeeded (so we know whether to restore on exit)."""
        if self.camera_manager is None:
            return False
        try:
            self.camera_manager.set_exposure(
                self.cam_idx, ae_enable=not lock, persist=False)
            return True
        except Exception as e:
            self._progress({"type": "backlash_progress", "phase": "ae_lock_failed",
                            "lock": lock, "error": str(e)})
            return False

    # ----- per (axis, direction) -----------------------------------------

    def _driver_for(self, axis: str):
        return self.pan_mc.driver if axis == "yaw" else self.tilt_mc.driver

    def _measure_one_trial_fit(self, axis: str, direction: str,
                               trial_idx: int, *,
                               do_texture_check: bool = False) -> _TrialFit:
        """Sweep `probe_steps` and linear-fit shift_mag = slope * N + intercept.

        Each probe is well past the expected backlash (smallest is N=20),
        so the measured shift is large and the linear fit is robust to
        per-probe noise. The x-intercept of the fit is the backlash steps;
        the slope is pixels of image motion per commanded step (a useful
        free byproduct calibration).

        Per-probe sequence (same as Iteration 3):
            engage -> ref frame -> probe N -> probe frame ->
            return (reverse_dir * (engage_steps - N))

        Texture-check fires on the very first probe of the very first
        trial for each (axis, direction), so the abort happens early if
        the scene lacks features. `do_texture_check=True` opts in.
        """
        engage_dir = +1 if direction == "pos" else -1
        reverse_dir = -engage_dir
        driver = self._driver_for(axis)
        prev_ramp = driver.ramping_enabled
        fit_points: list = []  # [(N, mag, dx, dy)]
        texture_checked = not do_texture_check

        # Pitch uses pitch_probe_steps when set, otherwise the default
        # probe_steps; yaw always uses probe_steps. This lets the caller
        # tune ranges separately because yaw and pitch usually have very
        # different gear reductions and therefore different signal-to-
        # backlash ratios.
        if axis == "pitch" and self._effective_pitch_probe_steps:
            probe_list = self._effective_pitch_probe_steps
        else:
            probe_list = self._effective_probe_steps

        try:
            for probe_idx, n in enumerate(probe_list, start=1):
                if self.cancel_event.is_set():
                    raise CalibrationCancelled()

                # Engage and capture reference.
                self._engage(axis, engage_dir)
                ref = self._capture_after_motion()
                if not texture_checked:
                    # Raises CalibrationAborted("low_features", ...) on fail.
                    self._texture_check(ref, context=f"{axis}_{direction}")
                    texture_checked = True

                # Probe: drive reverse_dir * n with ramping off (clean
                # short-distance motion profile).
                driver.ramping_enabled = False
                try:
                    _step_holding_enable(driver, reverse_dir * n)
                finally:
                    driver.ramping_enabled = prev_ramp
                time.sleep(self.settle_s)

                # Measure shift via KLT.
                frame = self._capture_fresh()
                dx, dy = self._phase_shift(ref, frame)
                mag = math.hypot(dx, dy)
                fit_points.append((int(n), float(mag), float(dx), float(dy)))

                self._progress({
                    "type": "backlash_progress", "phase": "probe",
                    "axis": axis, "direction": direction, "trial": trial_idx,
                    "n_steps": n, "shift_px": mag,
                    "dx": dx, "dy": dy,
                })
                self._record_probe_debug(
                    axis=axis, direction=direction, trial=trial_idx,
                    probe_idx=probe_idx, n_steps=n,
                    dx=dx, dy=dy, mag=mag, threshold=0.0,
                    ref=ref, probe=frame, phase="probe")

                # Return to start: continue reverse_dir for (engage_steps - n)
                # more. No new direction reversal — next probe's engage
                # handles it.
                return_steps = self.engage_steps - n
                if return_steps > 0:
                    driver.ramping_enabled = False
                    try:
                        _step_holding_enable(driver, reverse_dir * return_steps)
                    finally:
                        driver.ramping_enabled = prev_ramp
                    time.sleep(self.settle_s)
        finally:
            driver.ramping_enabled = prev_ramp

        # Drop probes where the tracker likely failed (returned ~zero shift
        # because the camera moved off-frame and no features tracked).
        # We keep the raw fit_points in the JSON so the user can see what
        # was discarded.
        fit_clean = [p for p in fit_points if p[1] >= 0.5]
        if len(fit_clean) < 2:
            self._progress({
                "type": "backlash_progress", "phase": "fit_insufficient",
                "axis": axis, "direction": direction, "trial": trial_idx,
                "kept": len(fit_clean), "total": len(fit_points),
            })
            return _TrialFit(0.0, 0.0, 0.0, 0.0, fit_points)
        ns = np.array([p[0] for p in fit_clean], dtype=float)
        mags = np.array([p[1] for p in fit_clean], dtype=float)
        # Piecewise fit: try each leading-K-probes-as-deadband split and
        # pick the lowest-SSE one. K=0 (no deadband) uses Theil-Sen for
        # outlier robustness. K>=1 uses least-squares on the linear
        # segment only. Handles three cases the pure linear fit couldn't:
        #   - pitch with zero deadband (chooses K=0)
        #   - yaw_neg with a single below-deadband probe (chooses K=1)
        #   - yaw_pos with a sharp release at large N (chooses K=N-2)
        slope, intercept, k_deadband, r_squared = _piecewise_fit(ns, mags)
        # Slope sanity check. If the data is essentially noise (no real
        # motion across any probe), `-intercept/slope` blows up or goes
        # negative. Report the user's max_steps as a "didn't measure"
        # sentinel and let _aggregate flag low_confidence.
        if abs(slope) < 0.01:
            self._progress({
                "type": "backlash_progress", "phase": "noise_dominated",
                "axis": axis, "direction": direction, "trial": trial_idx,
                "slope": float(slope),
            })
            return _TrialFit(
                backlash_steps=float(self.max_steps_user),
                slope_px_per_step=float(slope),
                r_squared=0.0,
                intercept_px=float(intercept),
                k_deadband=int(k_deadband),
                fit_data=fit_points,
            )
        backlash = -float(intercept) / float(slope)
        self._progress({
            "type": "backlash_progress", "phase": "fit_done",
            "axis": axis, "direction": direction, "trial": trial_idx,
            "backlash_steps": float(backlash),
            "slope_px_per_step": float(slope),
            "r_squared": float(r_squared),
            "k_deadband": int(k_deadband),
        })
        return _TrialFit(
            backlash_steps=float(backlash),
            slope_px_per_step=float(slope),
            r_squared=float(r_squared),
            intercept_px=float(intercept),
            k_deadband=int(k_deadband),
            fit_data=fit_points,
        )

    # ----- primitives ----------------------------------------------------

    def _engage(self, axis: str, engage_dir: int) -> None:
        """Drive `engage_steps` in `engage_dir` with ramping ON, settle."""
        driver = self._driver_for(axis)
        prev_ramp = driver.ramping_enabled
        driver.ramping_enabled = True
        try:
            self._progress({"type": "backlash_progress", "phase": "engaging",
                            "axis": axis, "direction":
                                "pos" if engage_dir > 0 else "neg"})
            _step_holding_enable(driver, engage_dir * self.engage_steps)
        finally:
            driver.ramping_enabled = prev_ramp
        time.sleep(self.settle_s)

    def _capture_fresh(self) -> np.ndarray:
        """Grab the latest frame from the background reader thread, convert
        to float32 grayscale. The reader is always pulling at full rate so
        no draining needed — the frame is at most ~one frame interval old."""
        frame = self._reader.latest() if self._reader is not None else None
        if frame is None:
            # Brief retry — the stream can momentarily stutter (e.g., after
            # a long settle the reader thread may have just hit a transient
            # read failure and be backing off).
            for _ in range(20):
                time.sleep(0.1)
                frame = self._reader.latest() if self._reader is not None else None
                if frame is not None:
                    break
        if frame is None:
            raise CalibrationAborted("rtsp_read_failed", self.rtsp_url)
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        return gray.astype(np.float32)

    def _capture_after_motion(self) -> np.ndarray:
        """Capture after the motion settle window already elapsed."""
        return self._capture_fresh()

    def _detect_features(self, frame: np.ndarray) -> np.ndarray:
        """goodFeaturesToTrack on a grayscale frame. Returns (N, 1, 2) float32
        of corner positions, or an empty array."""
        gray = frame if frame.dtype == np.float32 else frame.astype(np.float32)
        corners = cv2.goodFeaturesToTrack(gray, **self._gftt_params)
        if corners is None:
            return np.empty((0, 1, 2), dtype=np.float32)
        return corners

    def _estimate_shift(self, ref: np.ndarray, frame: np.ndarray) -> tuple:
        """Sub-pixel (dx, dy) shift from ref to frame via Lucas-Kanade
        optical flow on detected corners + MAD-based outlier rejection.

        KLT can return spuriously-large displacements when the true motion
        exceeds its effective search range (winSize x 2^maxLevel) — the
        tracker locks onto whatever nearby feature has similar appearance.
        We compute the global median, then drop any per-corner shift more
        than max(2px, 3*MAD) from it, then re-median the survivors. This
        prevents one or two rogue tracks (or many, in extreme cases) from
        dragging the answer.
        """
        corners = self._detect_features(ref)
        if corners.shape[0] < 4:
            self._last_tracks = None
            return 0.0, 0.0
        ref_u8 = np.clip(ref, 0, 255).astype(np.uint8)
        frame_u8 = np.clip(frame, 0, 255).astype(np.uint8)
        nxt, status, _err = cv2.calcOpticalFlowPyrLK(
            ref_u8, frame_u8, corners, None, **self._lk_params)
        if nxt is None or status is None:
            self._last_tracks = None
            return 0.0, 0.0
        ok = status.ravel().astype(bool)
        src = corners[ok].reshape(-1, 2)
        dst = nxt[ok].reshape(-1, 2)
        if src.shape[0] < 4:
            self._last_tracks = None
            return 0.0, 0.0
        deltas = dst - src

        # Initial median (robust to up to 50% outliers, but breaks past that).
        dx_med = float(np.median(deltas[:, 0]))
        dy_med = float(np.median(deltas[:, 1]))

        # Per-track Euclidean distance from the median displacement.
        dists = np.hypot(deltas[:, 0] - dx_med, deltas[:, 1] - dy_med)
        # MAD = median absolute deviation. 3*MAD is the standard "outlier"
        # threshold (≈ 2 sigma for gaussian noise); 2px floor stops us from
        # rejecting everything when real motion is sub-pixel.
        mad = float(np.median(dists))
        thresh = max(2.0, 3.0 * mad)
        inlier_mask = dists <= thresh

        if int(inlier_mask.sum()) < 4:
            # Not enough good tracks; fall back to the all-track median and
            # show every track in the debug view so the user can see it
            # really was a mess.
            self._last_tracks = (src, dst)
            return dx_med, dy_med

        src_in = src[inlier_mask]
        dst_in = dst[inlier_mask]
        deltas_in = dst_in - src_in
        dx = float(np.median(deltas_in[:, 0]))
        dy = float(np.median(deltas_in[:, 1]))
        # Show only inliers in the debug composite so the cyan tracks
        # reflect the consensus the algorithm actually trusted.
        self._last_tracks = (src_in, dst_in)
        return dx, dy

    # Backwards-compat alias for the rest of the module that still calls the
    # old name. Single source of truth lives in _estimate_shift.
    def _phase_shift(self, ref, frame):
        return self._estimate_shift(ref, frame)

    def _record_probe_debug(self, *, axis, direction, trial, probe_idx,
                            n_steps, dx, dy, mag, threshold, ref, probe,
                            phase) -> None:
        """Append a row to summary.json and (if debug_dir set) save the
        side-by-side composite PNG. No-op when debug is off."""
        if self.debug_dir is None:
            return
        row = {
            "phase": phase,
            "axis": axis, "direction": direction,
            "trial": trial, "probe_idx": probe_idx,
            "n_steps": n_steps,
            "dx": float(dx), "dy": float(dy), "mag": float(mag),
        }
        self._debug_rows.append(row)
        try:
            filename = (f"{axis}_{direction}_t{trial:02d}_"
                        f"p{probe_idx:03d}_n{n_steps:04d}.png")
            header = (f"{axis} {direction} trial={trial} probe={probe_idx} "
                      f"N={n_steps} mag={mag:.3f}px")
            self._save_debug_composite(ref, probe, dx, dy, mag, threshold,
                                       header, filename)
            row["file"] = filename
        except Exception as e:
            print(f"backlash_calibration: debug composite failed: {e!r}")

    def _save_debug_composite(self, ref, probe, dx, dy, mag, threshold,
                              header, filename) -> None:
        """Render reference | probe side-by-side with:
          - detected corners drawn as green circles on the ref half
          - per-corner motion vectors (src -> dst) drawn as cyan lines on
            the probe half
          - the *median* shift drawn from the probe center as a yellow
            arrow (the value actually used for the threshold decision)
        Pulls tracked features from self._last_tracks, populated by the
        most recent _estimate_shift call."""
        ref_u8 = np.clip(ref, 0, 255).astype(np.uint8)
        probe_u8 = np.clip(probe, 0, 255).astype(np.uint8)
        ref_bgr = cv2.cvtColor(ref_u8, cv2.COLOR_GRAY2BGR)
        probe_bgr = cv2.cvtColor(probe_u8, cv2.COLOR_GRAY2BGR)
        h, w = probe_u8.shape[:2]

        tracks = self._last_tracks
        n_tracks = 0 if tracks is None else int(tracks[0].shape[0])
        if tracks is not None:
            src, dst = tracks
            for (sx, sy) in src.astype(int):
                cv2.circle(ref_bgr, (int(sx), int(sy)), 3, (0, 255, 0), 1)
            # Draw per-corner motion on the probe half. Most of these are
            # near-identical when there's coherent motion; outliers stand
            # out visually.
            for (sx, sy), (ex, ey) in zip(src.astype(int), dst.astype(int)):
                cv2.circle(probe_bgr, (int(sx), int(sy)), 2, (0, 180, 180), 1)
                cv2.line(probe_bgr, (int(sx), int(sy)),
                         (int(ex), int(ey)), (200, 200, 0), 1)

        # Median shift arrow from probe center, amplified for visibility.
        target_px = 0.05 * w
        scale = target_px / max(mag, 0.5) if mag > 0 else 1.0
        scale = max(1.0, min(scale, 40.0))
        cx, cy = w // 2, h // 2
        end_x = max(0, min(w - 1, int(cx + dx * scale)))
        end_y = max(0, min(h - 1, int(cy + dy * scale)))
        # Above ~2px we consider the motion clearly visible — bright yellow
        # arrow. Below, grey, to make small shifts visually distinct from
        # large ones at a glance (was a hit/no-hit signal in the old
        # threshold-based algorithm; now it's just a visual aid).
        arrow_color = (0, 255, 255) if mag >= 2.0 else (180, 180, 180)
        if mag > 0.05:
            cv2.arrowedLine(probe_bgr, (cx, cy), (end_x, end_y),
                            arrow_color, 3, tipLength=0.25)
        else:
            cv2.circle(probe_bgr, (cx, cy), 4, arrow_color, 2)

        cv2.line(ref_bgr, (w - 1, 0), (w - 1, h - 1), (40, 40, 40), 1)
        composite = cv2.hconcat([ref_bgr, probe_bgr])
        cv2.rectangle(composite, (0, 0), (composite.shape[1], 30),
                      (0, 0, 0), -1)
        full_header = f"{header} | tracks={n_tracks}"
        cv2.putText(composite, full_header, (8, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(composite, "REF + corners", (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2,
                    cv2.LINE_AA)
        cv2.putText(composite, "PROBE + tracks", (w + 8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2,
                    cv2.LINE_AA)
        cv2.imwrite(os.path.join(self.debug_dir, filename), composite)

    def _texture_check(self, frame: np.ndarray, *,
                       context: str = "") -> None:
        """Feature-count gate. Replaces the old Laplacian-variance check
        (which collapsed on defocused scenes even when corners were still
        visible). Aborts with `low_features` if too few trackable corners.
        Saves the failing frame and an annotated copy with the detected
        corners drawn so the user can see exactly what the algorithm saw."""
        corners = self._detect_features(frame)
        n = int(corners.shape[0])
        if n < self.min_features:
            # Raw frame + an annotated copy showing whatever corners we DID
            # find — even a near-empty scene usually has a few, and seeing
            # them helps diagnose whether the issue is focus, exposure, or
            # genuinely a blank wall.
            self._save_raw_debug_frame(
                frame, f"low_features_{context or 'frame'}_n{n}.png")
            if self.debug_dir and n > 0:
                try:
                    u8 = np.clip(frame, 0, 255).astype(np.uint8)
                    bgr = cv2.cvtColor(u8, cv2.COLOR_GRAY2BGR)
                    for (px, py) in corners.reshape(-1, 2).astype(int):
                        cv2.circle(bgr, (int(px), int(py)), 4, (0, 255, 0), 1)
                    cv2.putText(bgr, f"{n} features detected",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.imwrite(
                        os.path.join(
                            self.debug_dir,
                            f"low_features_{context or 'frame'}_n{n}_annotated.png"),
                        bgr)
                except Exception as e:
                    print(f"backlash_calibration: annotated debug save failed: {e!r}")
            raise CalibrationAborted(
                "low_features",
                f"detected {n} trackable corners < {self.min_features}; "
                f"point at a scene with more edges/contrast or lower min_features")

    def _save_raw_debug_frame(self, frame: np.ndarray, filename: str) -> None:
        """Save a single grayscale frame as PNG into debug_dir. No-op if
        debug is off. Used for one-off diagnostic frames (initial capture,
        low-texture failure) that don't fit the composite pattern."""
        if not self.debug_dir:
            return
        try:
            u8 = np.clip(frame, 0, 255).astype(np.uint8)
            cv2.imwrite(os.path.join(self.debug_dir, filename), u8)
        except Exception as e:
            print(f"backlash_calibration: raw debug save failed ({filename}): {e!r}")

    # ----- aggregation ---------------------------------------------------

    def _aggregate(self, results: dict, warnings: list) -> dict:
        out = {}
        for key, ar in results.items():
            axis, direction = key.split("_")
            if not ar.trials:
                continue
            backlashes = [t.backlash_steps for t in ar.trials]
            slopes = [t.slope_px_per_step for t in ar.trials]
            r2s = [t.r_squared for t in ar.trials]
            bl_mean = float(np.mean(backlashes))
            bl_std = float(np.std(backlashes))
            sl_mean = float(np.mean(slopes))
            sl_std = float(np.std(slopes))
            r2_mean = float(np.mean(r2s))
            # When the absolute backlash is tiny, std/|mean| explodes for
            # purely numerical reasons (a ±1 step uncertainty on a 1-step
            # mean is "100% spread"). Treat this as a distinct semantic
            # state — "no deadband detected, no compensation needed" —
            # rather than as low-confidence. R² < 0.9 still flags
            # genuinely bad fits regardless of mean magnitude.
            near_zero_backlash = abs(bl_mean) < 5.0
            low_conf = bool(
                r2_mean < 0.9
                or (not near_zero_backlash
                    and abs(bl_mean) > 1e-6
                    and (bl_std / abs(bl_mean)) > 0.3)
            )

            # Cross-axis coupling warning. Inspect the *largest* probe's
            # (dx, dy) per trial — that's where the signal is biggest.
            shifts_largest = []
            for t in ar.trials:
                if not t.fit_data:
                    continue
                last_pt = max(t.fit_data, key=lambda p: p[0])  # max N
                shifts_largest.append((last_pt[2], last_pt[3]))  # (dx, dy)
            if shifts_largest:
                if axis == "yaw":
                    cross = sum(1 for dx, dy in shifts_largest if abs(dy) > abs(dx))
                else:
                    cross = sum(1 for dx, dy in shifts_largest if abs(dx) > abs(dy))
                if cross > len(shifts_largest) // 2:
                    warnings.append(
                        f"{key}: cross-axis component dominated on "
                        f"{cross}/{len(shifts_largest)} trials — possible "
                        f"mechanical coupling or mount tilt")

            out[key] = {
                "backlash_steps": {
                    "mean": bl_mean, "std": bl_std, "trials": backlashes,
                },
                "slope_px_per_step": {
                    "mean": sl_mean, "std": sl_std, "trials": slopes,
                },
                "r_squared_mean": r2_mean,
                "r_squared_trials": r2s,
                "k_deadband_trials": [int(t.k_deadband) for t in ar.trials],
                "low_confidence": low_conf,
                "near_zero_backlash": near_zero_backlash,
                "fit_data": [list(t.fit_data) for t in ar.trials],
                # Backwards-compat surface: legacy clients that read
                # `results[key].mean` for the click-to-move JS still work.
                "mean": bl_mean,
                "std": bl_std,
            }
        return out

    def _write_json(self, payload: dict) -> None:
        with open(self.output_path, "w") as f:
            json.dump(payload, f, indent=2)

    # ----- progress dispatch --------------------------------------------

    def _progress(self, msg: dict) -> None:
        try:
            self._progress_cb(msg)
        except Exception as e:
            # Don't let a broken callback kill the calibration.
            print(f"backlash_calibration: progress callback raised {e!r}")
