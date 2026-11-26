"""Pointing calibration: map (lat, lon, alt) targets to pan/tilt motor steps.

REBUILT 2026-11-25. The original module only existed on the rig's SD card and
was lost when the card corrupted. This version is a fresh implementation of
the interface websocket_server.py and index.html expect; it is not a copy
of the original, so check a few references on the rig before trusting
goto_latlon.

Model (one independent linear map per axis):
    pan_steps  = pan_steps_per_deg  * az_deg + pan_offset_steps
    tilt_steps = tilt_steps_per_deg * el_deg + tilt_offset_steps

az is degrees clockwise from true north, el is degrees above the horizon.
The slopes may be negative (they absorb the motor/direction sign). Azimuth
is unwrapped around the references' mean so a set of references that straddle
north still fits a straight line.

Ignored on purpose (fine for nearby landmarks, worth adding if the residual is
large): rig tilt/roll (the IMU's level offsets), non-perpendicular axes, and
atmospheric refraction.

Persisted as JSON next to this file (atomic tmp + rename), same pattern as
homography_calibration.py.
"""

import json
import math
import os
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import List, Optional, Tuple

# WGS84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s, c = math.sin(lat), math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * s * s)
    x = (n + alt_m) * c * math.cos(lon)
    y = (n + alt_m) * c * math.sin(lon)
    z = (n * (1.0 - _E2) + alt_m) * s
    return x, y, z


def earth_to_azel(obs_lat: float, obs_lon: float, obs_alt_m: float,
                  tgt_lat: float, tgt_lon: float, tgt_alt_m: float) -> Tuple[float, float]:
    """Azimuth (deg clockwise from true north, 0..360) and elevation (deg above
    the local horizon) of a target point as seen from the observer. Includes
    Earth curvature (WGS84 ellipsoid), no refraction."""
    ox, oy, oz = _geodetic_to_ecef(obs_lat, obs_lon, obs_alt_m)
    tx, ty, tz = _geodetic_to_ecef(tgt_lat, tgt_lon, tgt_alt_m)
    dx, dy, dz = tx - ox, ty - oy, tz - oz
    lat = math.radians(obs_lat)
    lon = math.radians(obs_lon)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    horiz = math.hypot(east, north)
    if horiz == 0.0 and up == 0.0:
        raise ValueError("target is at the observer's position")
    az = math.degrees(math.atan2(east, north)) % 360.0
    el = math.degrees(math.atan2(up, horiz))
    return az, el


def _wrap_near(az: float, center: float) -> float:
    """Return az + k*360 closest to center."""
    return center + ((az - center + 180.0) % 360.0 - 180.0)


@dataclass
class Observer:
    lat: float
    lon: float
    alt_m: float


@dataclass
class Reference:
    name: str
    lat: float
    lon: float
    alt_m: float
    pan_steps: int
    tilt_steps: int
    captured_at: str = ""


@dataclass
class PointingFit:
    pan_steps_per_deg: float
    pan_offset_steps: float
    tilt_steps_per_deg: float
    tilt_offset_steps: float
    az_center_deg: float          # azimuth branch the fit was made on
    rms_residual_deg: float
    n_references: int
    method: str = "least_squares"  # or "fixed_slopes"
    computed_at: str = ""
    residuals_deg: List[dict] = field(default_factory=list)


class PointingCalibration:
    def __init__(self, path: str):
        self.path = path
        self.observer: Optional[Observer] = None
        self.references: List[Reference] = []
        self.fit: Optional[PointingFit] = None
        self._load()

    # ---- persistence ------------------------------------------------------
    def _load(self) -> None:
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, "r") as f:
                d = json.load(f)
            if d.get("observer"):
                self.observer = Observer(**d["observer"])
            self.references = [Reference(**r) for r in d.get("references", [])]
            if d.get("fit"):
                self.fit = PointingFit(**d["fit"])
        except Exception as e:  # keep the server up on a bad file
            print(f"pointing calibration: failed to load {self.path}: {e}")

    def save(self) -> None:
        d = {
            "observer": asdict(self.observer) if self.observer else None,
            "references": [asdict(r) for r in self.references],
            "fit": asdict(self.fit) if self.fit else None,
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=2)
        os.replace(tmp, self.path)

    # ---- editing ----------------------------------------------------------
    def set_observer(self, lat: float, lon: float, alt_m: float) -> None:
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError("lat must be -90..90 and lon -180..180")
        self.observer = Observer(float(lat), float(lon), float(alt_m))
        # A fit made from another observer position is meaningless now.
        self.fit = None
        self.save()

    def add_reference(self, name: str, lat: float, lon: float, alt_m: float,
                      pan_steps: int, tilt_steps: int) -> None:
        self.references = [r for r in self.references if r.name != name]
        self.references.append(Reference(name, float(lat), float(lon), float(alt_m),
                                         int(pan_steps), int(tilt_steps), _utcnow_iso()))
        self.save()

    def remove_reference(self, name: str) -> bool:
        before = len(self.references)
        self.references = [r for r in self.references if r.name != name]
        if len(self.references) == before:
            return False
        self.save()
        return True

    # ---- fitting ----------------------------------------------------------
    def _ref_azel(self) -> List[Tuple[Reference, float, float]]:
        if not self.observer:
            raise RuntimeError("set the observer location first")
        o = self.observer
        out = []
        for r in self.references:
            az, el = earth_to_azel(o.lat, o.lon, o.alt_m, r.lat, r.lon, r.alt_m)
            out.append((r, az, el))
        if out:
            # Unwrap around the circular mean so references straddling north line up.
            sx = sum(math.sin(math.radians(a)) for _, a, _ in out)
            cx = sum(math.cos(math.radians(a)) for _, a, _ in out)
            center = math.degrees(math.atan2(sx, cx)) % 360.0
            out = [(r, _wrap_near(a, center), e) for r, a, e in out]
        return out

    @staticmethod
    def _line(xs: List[float], ys: List[float]) -> Tuple[float, float]:
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx < 1e-6:
            raise RuntimeError("references are too close together on this axis to fit a slope")
        k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        return k, my - k * mx

    def _finish(self, rows, kp, bp, kt, bt, method) -> PointingFit:
        res, sq = [], []
        for r, az, el in rows:
            pan_err = (r.pan_steps - (kp * az + bp)) / kp if kp else float("nan")
            tilt_err = (r.tilt_steps - (kt * el + bt)) / kt if kt else float("nan")
            res.append({"name": r.name, "pan_deg": pan_err, "tilt_deg": tilt_err,
                        "az_deg": az % 360.0, "el_deg": el})
            sq.append(pan_err ** 2 + tilt_err ** 2)
        rms = math.sqrt(sum(sq) / len(sq)) if sq else 0.0
        center = sum(az for _, az, _ in rows) / len(rows)
        self.fit = PointingFit(kp, bp, kt, bt, center, rms, len(rows), method,
                               _utcnow_iso(), res)
        self.save()
        return self.fit

    def compute(self) -> PointingFit:
        """Least-squares slope and offset per axis. Needs >=2 references that
        differ in both azimuth and elevation (>=3 gives a residual check)."""
        rows = self._ref_azel()
        if len(rows) < 2:
            raise RuntimeError(f"need at least 2 references, have {len(rows)}")
        kp, bp = self._line([az for _, az, _ in rows], [r.pan_steps for r, _, _ in rows])
        kt, bt = self._line([el for _, _, el in rows], [r.tilt_steps for r, _, _ in rows])
        return self._finish(rows, kp, bp, kt, bt, "least_squares")

    def compute_with_fixed_slopes(self, k_pan: float, k_tilt: float) -> PointingFit:
        """Use slopes measured elsewhere (IMU axis sweep, steps per degree) and
        only solve the offsets. Works with a single reference."""
        if not k_pan or not k_tilt:
            raise ValueError("slopes must be non-zero")
        rows = self._ref_azel()
        if not rows:
            raise RuntimeError("need at least 1 reference")
        bp = sum(r.pan_steps - k_pan * az for r, az, _ in rows) / len(rows)
        bt = sum(r.tilt_steps - k_tilt * el for r, _, el in rows) / len(rows)
        return self._finish(rows, float(k_pan), bp, float(k_tilt), bt, "fixed_slopes")

    # ---- using the fit ----------------------------------------------------
    def _need_fit(self) -> PointingFit:
        if not self.fit:
            raise RuntimeError("no pointing calibration yet (add references and compute)")
        if not self.observer:
            raise RuntimeError("no observer location set")
        return self.fit

    def azel_to_steps(self, az: float, el: float) -> Tuple[int, int]:
        f = self._need_fit()
        az = _wrap_near(az, f.az_center_deg)
        return (int(round(f.pan_steps_per_deg * az + f.pan_offset_steps)),
                int(round(f.tilt_steps_per_deg * el + f.tilt_offset_steps)))

    def latlon_to_steps(self, lat: float, lon: float, alt_m: float) -> Tuple[int, int, float, float]:
        self._need_fit()
        o = self.observer
        az, el = earth_to_azel(o.lat, o.lon, o.alt_m, lat, lon, alt_m)
        pan, tilt = self.azel_to_steps(az, el)
        return pan, tilt, az, el

    def steps_to_azel(self, pan_steps: float, tilt_steps: float) -> Tuple[float, float]:
        f = self._need_fit()
        az = (pan_steps - f.pan_offset_steps) / f.pan_steps_per_deg
        el = (tilt_steps - f.tilt_offset_steps) / f.tilt_steps_per_deg
        return az % 360.0, el

    def state_dict(self) -> dict:
        f = self.fit
        return {
            "observer": asdict(self.observer) if self.observer else None,
            "references": [asdict(r) for r in self.references],
            "calibration": ({
                "pan_steps_per_deg": f.pan_steps_per_deg,
                "pan_offset_steps": f.pan_offset_steps,
                "tilt_steps_per_deg": f.tilt_steps_per_deg,
                "tilt_offset_steps": f.tilt_offset_steps,
                "rms_residual_deg": f.rms_residual_deg,
                "n_references": f.n_references,
                "method": f.method,
                "computed_at": f.computed_at,
                "residuals_deg": f.residuals_deg,
            } if f else None),
        }
