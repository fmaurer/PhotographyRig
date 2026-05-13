"""On-demand homography calibration between the wide and zoom cameras.

Fits a planar homography mapping points in the zoom frame to points in the
wide frame, projects the zoom frame's four corners through the fit to produce
a "red box" overlay on the wide stream, and persists the result to JSON so
the overlay re-appears on next page load.

Mirrors the pattern in pointing_calibration.py:
  - @dataclass fit result
  - JSON load/save (atomic via tmp + rename)
  - state_dict() for the websocket payload
  - rms_residual_* as the confidence signal

The IMU pose at fit time is stored but not yet acted on. One IMU can't observe
the relative pose between the two cameras (rig flex), so it's a side channel
for future flex-vs-pose modelling, not the calibration sensor.
"""

import json
import os
import traceback
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Optional

import cv2
import numpy as np


_MIN_MATCHES = 8           # cv2.findHomography needs >=4; we want margin.
_LOWE_RATIO = 0.75
_RANSAC_REPROJ_PX = 3.0


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass
class HomographyFit:
    H: list                       # 3x3 row-major, maps FULL-zoom -> FULL-wide pixel coords
    corners_wide_px: list         # [[x,y]*4] in wide-frame pixel space, TL,TR,BR,BL of the zoom
    zoom_size: list               # [w, h] of the zoom frame the H was fit on
    wide_size: list               # [w, h] of the wide frame the H was fit on
    inliers: int
    n_matches: int
    rms_px: float                 # reprojection RMS over RANSAC inliers (matching-space pixels)
    detector: str                 # "SIFT" or "AKAZE"
    undistorted: bool
    imu_pose: Optional[dict]      # snapshot at fit time (pitch/roll/yaw/timestamp), or None
    # The next three record the crop+scales applied during matching. They're
    # the recipe for reproducing the fit and the seed for the next auto-crop.
    # None for any means "wasn't applied".
    wide_crop: Optional[list] = None    # [x, y, w, h] in full-wide pixels
    wide_scale: Optional[list] = None   # [sx, sy] upsample applied to cropped wide (legacy path)
    zoom_scale: Optional[list] = None   # [k, k] downsample applied to the zoom before matching
    # ECC-mode score in [-1, 1]; 1 is perfect intensity correlation.
    # None when method != "ecc".
    ecc_cc: Optional[float] = None
    method: str = "features"
    computed_at: str = field(default_factory=_utcnow_iso)


class HomographyCalibration:
    def __init__(self, path: str):
        self.path = path
        self.fit: Optional[HomographyFit] = None
        self._load()

    # ---- persistence -------------------------------------------------------

    def _load(self) -> None:
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, "r") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            print(f"HomographyCalibration: failed to load {self.path}: {e}")
            return
        if data:
            try:
                self.fit = HomographyFit(**data)
            except TypeError as e:
                # Older/incompatible JSON shape — start fresh rather than crash.
                print(f"HomographyCalibration: incompatible JSON, ignoring: {e}")

    def save(self) -> None:
        data = asdict(self.fit) if self.fit else None
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, self.path)

    def state_dict(self) -> Optional[dict]:
        return asdict(self.fit) if self.fit else None

    # ---- the fit -----------------------------------------------------------

    def compute(
        self,
        frame_wide: np.ndarray,
        frame_zoom: np.ndarray,
        imu_pose: Optional[dict] = None,
        *,
        wide_crop: Optional[tuple] = None,        # (x, y, w, h) in full-wide pixels
        equalize_scale: bool = True,              # downsample zoom to ~wide_crop resolution
        zoom_extra_scale: float = 1.0,            # multiplier on auto k_z; <1 = more aggressive downsample
        method: str = "features",                 # "features" or "ecc"
        match_blur_sigma: Optional[float] = None, # zoom-side Gaussian sigma; None = auto from wide_scale
        undistort: bool = False,
        K_wide: Optional[np.ndarray] = None,
        D_wide: Optional[np.ndarray] = None,
        K_zoom: Optional[np.ndarray] = None,
        D_zoom: Optional[np.ndarray] = None,
        debug_dir: Optional[str] = None,
    ) -> HomographyFit:
        """Detect + match features in both frames, fit H (zoom -> wide), project corners.

        For high zoom ratios, pass `wide_crop=(x, y, w, h)` to restrict feature
        matching to the rough zoom-aim area in the wide frame — and leave
        `equalize_scale=True` so the cropped region is upsampled to roughly the
        zoom's resolution before matching.

        The returned H and corners are always expressed in full-zoom and
        full-wide pixel coordinates regardless of any crop/scale applied
        during matching — the composition is undone before returning.

        If `debug_dir` is set, intermediate artifacts (input frames, the cropped+
        resized "working" wide, keypoint overlays, raw and inlier match
        visualizations, and a metadata.json) are written there. Artifacts are
        written even on failure so you can diagnose what went wrong.
        """
        if method not in ("features", "ecc"):
            raise ValueError(f"method must be 'features' or 'ecc', got {method!r}")

        # Everything the finally block needs to write debug artifacts.
        dbg = {
            "ok": False,
            "params": {
                "wide_crop_input": list(wide_crop) if wide_crop else None,
                "equalize_scale": bool(equalize_scale),
                "zoom_extra_scale": float(zoom_extra_scale),
                "method": method,
                "match_blur_sigma": match_blur_sigma,
                "undistort": bool(undistort),
            },
            "stages": [],  # ordered list of stage names reached, for debugging
            "error": None,
        } if debug_dir else None

        try:
            return self._compute_impl(
                frame_wide, frame_zoom, imu_pose,
                wide_crop=wide_crop, equalize_scale=equalize_scale,
                zoom_extra_scale=float(zoom_extra_scale),
                method=method, match_blur_sigma=match_blur_sigma,
                undistort=undistort,
                K_wide=K_wide, D_wide=D_wide, K_zoom=K_zoom, D_zoom=D_zoom,
                dbg=dbg,
            )
        except Exception as e:
            if dbg is not None:
                dbg["error"] = {
                    "type": type(e).__name__,
                    "message": str(e),
                    "traceback": traceback.format_exc(),
                }
            raise
        finally:
            if debug_dir is not None:
                try:
                    _write_debug_artifacts(debug_dir, dbg)
                except Exception as e:
                    # Don't let debug writing mask the real result/exception.
                    print(f"HomographyCalibration: failed to write debug to {debug_dir}: {e}")

    def _compute_impl(self, frame_wide, frame_zoom, imu_pose, *,
                      wide_crop, equalize_scale, zoom_extra_scale,
                      method, match_blur_sigma,
                      undistort, K_wide, D_wide, K_zoom, D_zoom, dbg):
        def record(stage, **kwargs):
            if dbg is not None:
                dbg["stages"].append(stage)
                for k, v in kwargs.items():
                    dbg[k] = v

        gray_w_full = _to_gray(frame_wide)
        gray_z = _to_gray(frame_zoom)

        if undistort:
            if K_wide is None or D_wide is None or K_zoom is None or D_zoom is None:
                raise RuntimeError("undistort=True requires K/D for both cameras")
            gray_w_full = cv2.undistort(gray_w_full, np.asarray(K_wide), np.asarray(D_wide))
            gray_z = cv2.undistort(gray_z, np.asarray(K_zoom), np.asarray(D_zoom))

        record("captured", gray_w_full=gray_w_full, gray_z=gray_z)

        full_ww, full_wh = int(gray_w_full.shape[1]), int(gray_w_full.shape[0])
        full_zw, full_zh = int(gray_z.shape[1]), int(gray_z.shape[0])

        # --- crop the wide ---
        crop_offset = (0, 0)
        applied_crop = None  # what we'll persist; None if no crop applied
        work_w = gray_w_full
        if wide_crop is not None:
            cx, cy, cw, ch = (int(round(v)) for v in wide_crop)
            cx = max(0, min(full_ww - 1, cx))
            cy = max(0, min(full_wh - 1, cy))
            cw = max(1, min(full_ww - cx, cw))
            ch = max(1, min(full_wh - cy, ch))
            work_w = gray_w_full[cy:cy + ch, cx:cx + cw]
            crop_offset = (cx, cy)
            applied_crop = [cx, cy, cw, ch]

        record("cropped", applied_crop=applied_crop, crop_offset=crop_offset, cropped_wide=work_w.copy())

        # --- equalise scale: uniformly DOWNSAMPLE the zoom to roughly the
        #     wide_crop's native dimensions. INTER_AREA gives proper Nyquist
        #     anti-aliasing, so the downsampled zoom retains real low-frequency
        #     content matching what the wide_crop natively contains — no
        #     synthetic interpolation, no blur artifacts, both images at the
        #     same effective resolution.
        #     This replaces the earlier "upsample wide + blur zoom" approach,
        #     which couldn't add real information to the wide and induced
        #     gradient mismatch between the two views. ---
        ww_w, ww_h = work_w.shape[1], work_w.shape[0]
        k_z = 1.0
        applied_zoom_scale = None
        if equalize_scale:
            # Auto: uniform "fit inside" — zoom_small fits inside work_w on both axes.
            auto_k_z = min(ww_w / full_zw, ww_h / full_zh)
            # User can multiply this further (zoom_extra_scale < 1 means more
            # aggressive downsample; useful when the real zoom FOV is narrower
            # than the wide_crop's pixel-extent would suggest, i.e. the zoom
            # actually sees a small fraction of the cropped wide region).
            k_z = auto_k_z * float(zoom_extra_scale)
            if k_z < 0.999:
                new_zw = max(1, int(round(full_zw * k_z)))
                new_zh = max(1, int(round(full_zh * k_z)))
                gray_z_match = cv2.resize(gray_z, (new_zw, new_zh),
                                          interpolation=cv2.INTER_AREA)
                applied_zoom_scale = [float(k_z), float(k_z)]
            else:
                # Either no downsample needed or user explicitly upscaled past 1.
                k_z = 1.0
                gray_z_match = gray_z
        else:
            gray_z_match = gray_z

        # Optional explicit zoom-side blur (kept as a knob; off by default since
        # INTER_AREA already bandlimits properly).
        applied_blur = None
        if match_blur_sigma is not None and float(match_blur_sigma) > 0.3:
            gray_z_match = cv2.GaussianBlur(
                gray_z_match, (0, 0),
                sigmaX=float(match_blur_sigma), sigmaY=float(match_blur_sigma),
            )
            applied_blur = float(match_blur_sigma)

        record("scaled", applied_zoom_scale=applied_zoom_scale,
               applied_blur=applied_blur, k_z=k_z,
               gray_z_match=gray_z_match, work_w=work_w)

        # H_raw maps full_zoom -> work_w (cropped+scaled wide). Both methods
        # populate this; the rest of the pipeline (composition + corners) is
        # then identical.
        inliers = 0
        n_matches = 0
        rms_px = 0.0
        ecc_cc = None
        detector_name = ""
        mask = None

        if method == "features":
            # ---- feature matching path ----
            match_result = _detect_and_match_verbose(gray_z_match, work_w)
            kp_z, kp_w = match_result["kp_src"], match_result["kp_dst"]
            raw_pairs = match_result["raw_matches"]
            detector_name = match_result["detector"]
            src_pts = match_result["src_pts"]
            dst_pts = match_result["dst_pts"]
            n_matches = len(src_pts)

            record("matched",
                   detector=detector_name,
                   n_keypoints_zoom=len(kp_z),
                   n_keypoints_wide=len(kp_w),
                   n_matches=n_matches,
                   kp_z=kp_z, kp_w=kp_w, raw_pairs=raw_pairs)

            if n_matches < _MIN_MATCHES:
                raise RuntimeError(
                    f"insufficient matches: {n_matches} < {_MIN_MATCHES} "
                    f"(detector={detector_name}, crop={applied_crop}, scale={applied_scale}) "
                    f"— try a tighter wide_crop, a more textured scene, or the ECC method"
                )

            H_raw, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, _RANSAC_REPROJ_PX)
            if H_raw is None or not np.all(np.isfinite(H_raw)):
                raise RuntimeError(f"degenerate homography (detector={detector_name}, n={n_matches})")

            inliers = int(mask.sum())
            record("ransac", H_raw=H_raw, mask=mask, inliers=inliers)
            if inliers < 4:
                raise RuntimeError(
                    f"too few inliers after RANSAC: {inliers} (detector={detector_name}, n={n_matches})"
                )
            rms_px = _rms_inliers(src_pts, dst_pts, H_raw, mask)

        else:
            # ---- ECC alignment path ----
            # findTransformECC needs same-size template & input. We now pad the
            # (downsampled) zoom to work_w's dims rather than padding work_w,
            # since work_w is the "reference" and zoom_small fits inside it
            # after the uniform downsample.
            zh_m, zw_m = gray_z_match.shape[:2]
            pad_x_l = max(0, (ww_w - zw_m) // 2)
            pad_y_t = max(0, (ww_h - zh_m) // 2)
            pad_x_r = max(0, ww_w - zw_m - pad_x_l)
            pad_y_b = max(0, ww_h - zh_m - pad_y_t)
            if (pad_x_l or pad_x_r or pad_y_t or pad_y_b):
                padded_z = cv2.copyMakeBorder(
                    gray_z_match, pad_y_t, pad_y_b, pad_x_l, pad_x_r,
                    cv2.BORDER_CONSTANT, value=int(gray_z_match.mean()),
                )
            else:
                padded_z = gray_z_match
            # Safety net: if zoom_small is somehow bigger than work_w on either
            # axis (e.g. equalize_scale was off and wide_crop is tiny), resize
            # zoom_small down so dims match.
            if padded_z.shape[:2] != (ww_h, ww_w):
                padded_z = cv2.resize(padded_z, (ww_w, ww_h), interpolation=cv2.INTER_AREA)
                pad_x_l = pad_y_t = 0

            # Initial warp: seed from cached H if available. The warp here maps
            # padded_z -> work_w (== padded_z -> cropped_wide). We convert the
            # cached full_zoom -> full_wide H into that space:
            #   padded_z -> zoom_small  : T_unpad_zoom (translate by -(pad_x_l, pad_y_t))
            #   zoom_small -> full_zoom : S_zoom_up   (multiply by 1/k_z)
            #   full_zoom  -> full_wide : H_cached
            #   full_wide  -> work_w    : T_uncrop    (translate by -(cx, cy))
            warp = np.eye(3, dtype=np.float32)
            if self.fit is not None and self.fit.H is not None and \
                    self.fit.wide_size == [full_ww, full_wh] and \
                    self.fit.zoom_size == [full_zw, full_zh]:
                try:
                    H_cached = np.array(self.fit.H, dtype=np.float64)
                    inv_k = 1.0 / k_z if k_z != 0 else 1.0
                    T_unpad_zoom = np.array([[1, 0, -float(pad_x_l)],
                                             [0, 1, -float(pad_y_t)],
                                             [0, 0, 1.0]])
                    S_zoom_up = np.array([[inv_k, 0, 0],
                                          [0, inv_k, 0],
                                          [0, 0, 1.0]])
                    T_uncrop = np.array([[1, 0, -float(crop_offset[0])],
                                         [0, 1, -float(crop_offset[1])],
                                         [0, 0, 1.0]])
                    warp = (T_uncrop @ H_cached @ S_zoom_up @ T_unpad_zoom).astype(np.float32)
                except Exception:
                    warp = np.eye(3, dtype=np.float32)

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 500, 1e-5)
            try:
                cc, warp = cv2.findTransformECC(
                    templateImage=work_w,
                    inputImage=padded_z,
                    warpMatrix=warp,
                    motionType=cv2.MOTION_HOMOGRAPHY,
                    criteria=criteria,
                    inputMask=None,
                    gaussFiltSize=5,
                )
            except cv2.error as e:
                raise RuntimeError(
                    f"ECC failed to converge: {e} "
                    f"(crop={applied_crop}, zoom_scale={applied_zoom_scale}) "
                    f"— try a tighter wide_crop closer to the zoom-aim area"
                )

            ecc_cc = float(cc)
            # warp maps padded_z (input) -> work_w (template). Un-pad on the zoom
            # side so the rest of the composition gets H_raw : zoom_small -> work_w.
            T_pad_zoom = np.array([[1, 0, float(pad_x_l)],
                                   [0, 1, float(pad_y_t)],
                                   [0, 0, 1.0]])
            H_raw = warp.astype(np.float64) @ T_pad_zoom
            detector_name = "ECC"
            record("ecc", H_raw=H_raw, ecc_cc=ecc_cc, padded_z=padded_z,
                   pad_offset=[pad_x_l, pad_y_t])

        # --- compose H to express it in full-zoom -> full-wide coordinates ---
        # H_raw maps zoom_small -> work_w (== cropped_wide; no wide upsample now).
        # To reach full_zoom on the left and full_wide on the right:
        #     full_zoom --S_zoom_down--> zoom_small --H_raw--> cropped_wide --T_crop--> full_wide
        #     H_full = T_crop @ H_raw @ S_zoom_down
        S_zoom_down = np.array([[k_z, 0.0, 0.0],
                                [0.0, k_z, 0.0],
                                [0.0, 0.0, 1.0]], dtype=np.float64)
        T_crop = np.array([[1.0, 0.0, float(crop_offset[0])],
                           [0.0, 1.0, float(crop_offset[1])],
                           [0.0, 0.0, 1.0]], dtype=np.float64)
        H_full = T_crop @ H_raw @ S_zoom_down

        corners = _project_corners(H_full, full_zw, full_zh)

        self.fit = HomographyFit(
            H=[list(map(float, row)) for row in H_full],
            corners_wide_px=[[float(x), float(y)] for x, y in corners],
            zoom_size=[full_zw, full_zh],
            wide_size=[full_ww, full_wh],
            inliers=inliers,
            n_matches=int(n_matches),
            rms_px=float(rms_px),
            detector=detector_name,
            undistorted=bool(undistort),
            imu_pose=dict(imu_pose) if imu_pose else None,
            wide_crop=applied_crop,
            wide_scale=None,                # no longer applies in the downsample-zoom scheme
            zoom_scale=applied_zoom_scale,
            ecc_cc=ecc_cc,
            method=method,
        )
        if dbg is not None:
            dbg["ok"] = True
            dbg["rms_px"] = float(rms_px)
            dbg["ecc_cc"] = ecc_cc
            dbg["fit"] = asdict(self.fit)
            dbg["H_full"] = H_full.tolist()
        return self.fit


    # -----------------------------------------------------------------------
    # manual point-pair path (no auto-matching; bulletproof for tricky scenes)
    # -----------------------------------------------------------------------

    def compute_from_pairs(
        self,
        wide_pts,                    # list of [x, y] in full-wide pixel coords
        zoom_pts,                    # list of [x, y] in full-zoom pixel coords (same length)
        full_zoom_size,              # [w, h]
        full_wide_size,              # [w, h]
        imu_pose: Optional[dict] = None,
        debug_dir: Optional[str] = None,
    ) -> HomographyFit:
        """Compute H directly from user-supplied point correspondences.

        With exactly 4 pairs we use cv2.getPerspectiveTransform (exact solve).
        With 5+ pairs we use cv2.findHomography with RANSAC at 5 px reproj
        threshold (forgives slightly imprecise clicks).
        """
        if len(wide_pts) != len(zoom_pts):
            raise RuntimeError(
                f"point-pair count mismatch: {len(wide_pts)} wide vs {len(zoom_pts)} zoom"
            )
        n = len(wide_pts)
        if n < 4:
            raise RuntimeError(f"need at least 4 point pairs, got {n}")

        src = np.array(zoom_pts, dtype=np.float32).reshape(-1, 1, 2)
        dst = np.array(wide_pts, dtype=np.float32).reshape(-1, 1, 2)

        if n == 4:
            H = cv2.getPerspectiveTransform(src.reshape(-1, 2), dst.reshape(-1, 2))
            mask = np.ones(n, dtype=np.uint8)
        else:
            H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)

        if H is None or not np.all(np.isfinite(H)):
            raise RuntimeError("degenerate homography from manual pairs")

        # RMS over the inliers.
        inlier_mask = mask.ravel().astype(bool)
        inliers = int(inlier_mask.sum())
        if inliers > 0:
            projected = cv2.perspectiveTransform(src[inlier_mask], H)
            diffs = projected - dst[inlier_mask]
            sq = (diffs[..., 0] ** 2 + diffs[..., 1] ** 2).ravel()
            rms_px = float(np.sqrt(sq.mean()))
        else:
            rms_px = float("inf")

        full_zw, full_zh = int(full_zoom_size[0]), int(full_zoom_size[1])
        full_ww, full_wh = int(full_wide_size[0]), int(full_wide_size[1])
        corners = _project_corners(H, full_zw, full_zh)

        self.fit = HomographyFit(
            H=[list(map(float, row)) for row in H],
            corners_wide_px=[[float(x), float(y)] for x, y in corners],
            zoom_size=[full_zw, full_zh],
            wide_size=[full_ww, full_wh],
            inliers=inliers,
            n_matches=n,
            rms_px=rms_px,
            detector="manual_pairs",
            undistorted=False,
            imu_pose=dict(imu_pose) if imu_pose else None,
            wide_crop=None,
            wide_scale=None,
            zoom_scale=None,
            ecc_cc=None,
            method="pairs",
        )

        if debug_dir is not None:
            try:
                os.makedirs(debug_dir, exist_ok=True)
                with open(os.path.join(debug_dir, "metadata.json"), "w") as f:
                    json.dump({
                        "ok": True,
                        "method": "pairs",
                        "n_pairs": n,
                        "inliers": inliers,
                        "rms_px": rms_px,
                        "wide_pts": [list(map(float, p)) for p in wide_pts],
                        "zoom_pts": [list(map(float, p)) for p in zoom_pts],
                        "fit": asdict(self.fit),
                        "computed_at": _utcnow_iso(),
                    }, f, indent=2, default=str)
            except Exception as e:
                print(f"compute_from_pairs: debug write failed: {e}")

        return self.fit


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _to_gray(img: np.ndarray) -> np.ndarray:
    if img is None:
        raise RuntimeError("frame is None")
    if img.ndim == 2:
        return img
    if img.ndim == 3 and img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    raise RuntimeError(f"unsupported frame shape {img.shape}")


def _detect_and_match_verbose(img_src: np.ndarray, img_dst: np.ndarray) -> dict:
    """Verbose version of _detect_and_match that returns keypoints and raw matches.

    Used by the debug path so we can visualise what the detector saw even when
    the fit ultimately fails. Falls back from SIFT to AKAZE on the same logic
    as _detect_and_match.
    """
    def _try(detector, name, norm):
        kp1, des1 = detector.detectAndCompute(img_src, None)
        kp2, des2 = detector.detectAndCompute(img_dst, None)
        if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
            return {
                "detector": name, "kp_src": kp1 or [], "kp_dst": kp2 or [],
                "raw_matches": [], "src_pts": np.empty((0, 1, 2), np.float32),
                "dst_pts": np.empty((0, 1, 2), np.float32),
            }
        matcher = cv2.BFMatcher(norm)
        good = []
        try:
            raw = matcher.knnMatch(des1, des2, k=2)
        except cv2.error:
            raw = []
        for pair in raw:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < _LOWE_RATIO * n.distance:
                good.append(m)
        if good:
            src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        else:
            src = np.empty((0, 1, 2), np.float32)
            dst = np.empty((0, 1, 2), np.float32)
        return {
            "detector": name, "kp_src": kp1, "kp_dst": kp2,
            "raw_matches": good, "src_pts": src, "dst_pts": dst,
        }

    sift_create = getattr(cv2, "SIFT_create", None)
    if sift_create is not None:
        try:
            res = _try(sift_create(), "SIFT", cv2.NORM_L2)
            if len(res["raw_matches"]) >= _MIN_MATCHES:
                return res
        except cv2.error:
            pass
    return _try(cv2.AKAZE_create(), "AKAZE", cv2.NORM_HAMMING)


def _detect_and_match(img_src: np.ndarray, img_dst: np.ndarray):
    """Detect features in both images and return matched point arrays.

    Returns (src_pts (N,1,2), dst_pts (N,1,2), detector_name).
    src_pts come from img_src; dst_pts from img_dst. So if you call this with
    (zoom, wide), the resulting findHomography(src, dst, ...) gives H_zoom_to_wide.

    Tries SIFT first (needs opencv-contrib-python; better at large scale gaps),
    falls back to AKAZE (vanilla opencv-python).
    """
    sift_create = getattr(cv2, "SIFT_create", None)
    if sift_create is not None:
        try:
            det = sift_create()
            kp1, des1 = det.detectAndCompute(img_src, None)
            kp2, des2 = det.detectAndCompute(img_dst, None)
            if des1 is not None and des2 is not None and len(kp1) >= 4 and len(kp2) >= 4:
                matcher = cv2.BFMatcher(cv2.NORM_L2)
                src, dst = _ratio_test(matcher, kp1, des1, kp2, des2)
                if len(src) >= _MIN_MATCHES:
                    return src, dst, "SIFT"
        except cv2.error:
            pass  # contrib not actually present; fall through to AKAZE

    det = cv2.AKAZE_create()
    kp1, des1 = det.detectAndCompute(img_src, None)
    kp2, des2 = det.detectAndCompute(img_dst, None)
    if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
        return np.empty((0, 1, 2), dtype=np.float32), np.empty((0, 1, 2), dtype=np.float32), "AKAZE"
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    src, dst = _ratio_test(matcher, kp1, des1, kp2, des2)
    return src, dst, "AKAZE"


def _ratio_test(matcher, kp1, des1, kp2, des2):
    raw = matcher.knnMatch(des1, des2, k=2)
    good = []
    for pair in raw:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < _LOWE_RATIO * n.distance:
            good.append(m)
    if not good:
        return np.empty((0, 1, 2), dtype=np.float32), np.empty((0, 1, 2), dtype=np.float32)
    src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    return src, dst


def _project_corners(H: np.ndarray, w: int, h: int):
    """Project the rectangle (0,0),(w,0),(w,h),(0,h) through H.

    Returned order is TL, TR, BR, BL — matches the input ordering so the JS
    can draw a closed polygon directly.
    """
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(corners, H)
    return [(float(p[0][0]), float(p[0][1])) for p in projected]


def _rms_inliers(src_pts: np.ndarray, dst_pts: np.ndarray, H: np.ndarray, mask) -> float:
    inlier_mask = mask.ravel().astype(bool)
    if not inlier_mask.any():
        return float("inf")
    src_in = src_pts[inlier_mask]
    dst_in = dst_pts[inlier_mask]
    projected = cv2.perspectiveTransform(src_in, H)
    diffs = projected - dst_in
    sq = (diffs[..., 0] ** 2 + diffs[..., 1] ** 2).ravel()
    return float(np.sqrt(sq.mean()))


# ---------------------------------------------------------------------------
# debug-artifact writer
# ---------------------------------------------------------------------------

# Quality knob for cv2.imwrite('.jpg', ...). 85 is a reasonable balance of size
# vs detail — fine for visual diagnosis, not lossless.
_DEBUG_JPEG_QUALITY = 85


def _write_debug_artifacts(debug_dir: str, dbg: Optional[dict]) -> None:
    """Save whatever stages were reached in `dbg` as JPEGs + metadata.json.

    Called from compute()'s finally block — safe to call when partial state
    is present (e.g. fit failed at matching stage and there are no inliers).
    """
    if dbg is None:
        return
    os.makedirs(debug_dir, exist_ok=True)
    write = lambda name, img: cv2.imwrite(
        os.path.join(debug_dir, name), img,
        [int(cv2.IMWRITE_JPEG_QUALITY), _DEBUG_JPEG_QUALITY],
    )

    metadata = {
        "ok": dbg.get("ok", False),
        "stages_reached": dbg.get("stages", []),
        "params": dbg.get("params", {}),
        "computed_at": _utcnow_iso(),
    }

    # ---- raw captures ----
    if "gray_w_full" in dbg:
        write("wide.jpg", dbg["gray_w_full"])
        metadata["wide_size"] = [int(dbg["gray_w_full"].shape[1]), int(dbg["gray_w_full"].shape[0])]
    if "gray_z" in dbg:
        write("zoom.jpg", dbg["gray_z"])
        metadata["zoom_size"] = [int(dbg["gray_z"].shape[1]), int(dbg["gray_z"].shape[0])]

    # ---- crop overlay on the full wide ----
    if "gray_w_full" in dbg and dbg.get("applied_crop"):
        bgr = cv2.cvtColor(dbg["gray_w_full"], cv2.COLOR_GRAY2BGR)
        cx, cy, cw, ch = dbg["applied_crop"]
        cv2.rectangle(bgr, (cx, cy), (cx + cw, cy + ch), (0, 255, 255), 3)
        write("wide_with_crop.jpg", bgr)
        metadata["wide_crop"] = dbg["applied_crop"]
    if "cropped_wide" in dbg:
        write("wide_cropped.jpg", dbg["cropped_wide"])

    # ---- working wide (what the detector actually saw) ----
    if "work_w" in dbg:
        # Named "wide_work" for continuity even though it's now just the native
        # cropped wide (no upsample). Compare side-by-side with zoom_small.jpg.
        write("wide_work.jpg", dbg["work_w"])
        metadata["work_w_size"] = [int(dbg["work_w"].shape[1]), int(dbg["work_w"].shape[0])]

    # ---- downsampled zoom (the matching companion to wide_work) ----
    if "gray_z_match" in dbg and dbg["gray_z_match"] is not None:
        gz = dbg["gray_z_match"]
        # Only emit a separate file if it actually differs from the raw zoom.
        if "gray_z" not in dbg or gz.shape != dbg["gray_z"].shape:
            write("zoom_small.jpg", gz)
            metadata["zoom_small_size"] = [int(gz.shape[1]), int(gz.shape[0])]
    metadata["zoom_scale"] = dbg.get("applied_zoom_scale")
    metadata["match_blur_sigma"] = dbg.get("applied_blur")

    # ---- keypoint overlays (drawn on the same image the detector saw) ----
    # gray_z_match is what features were detected on; fall back to gray_z if
    # equalize_scale was off or skipped.
    zoom_for_draw = dbg.get("gray_z_match") if dbg.get("gray_z_match") is not None else dbg.get("gray_z")
    if "kp_z" in dbg and zoom_for_draw is not None:
        kp_img = cv2.drawKeypoints(
            cv2.cvtColor(zoom_for_draw, cv2.COLOR_GRAY2BGR),
            dbg["kp_z"], None,
            color=(0, 200, 255),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        write("keypoints_zoom.jpg", kp_img)
        metadata["n_keypoints_zoom"] = int(dbg.get("n_keypoints_zoom", len(dbg["kp_z"])))
    if "kp_w" in dbg and "work_w" in dbg:
        kp_img = cv2.drawKeypoints(
            cv2.cvtColor(dbg["work_w"], cv2.COLOR_GRAY2BGR),
            dbg["kp_w"], None,
            color=(0, 200, 255),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        write("keypoints_wide.jpg", kp_img)
        metadata["n_keypoints_wide"] = int(dbg.get("n_keypoints_wide", len(dbg["kp_w"])))

    # ---- raw matches (post-ratio-test, pre-RANSAC) ----
    if "kp_z" in dbg and "kp_w" in dbg and "raw_pairs" in dbg \
            and zoom_for_draw is not None and "work_w" in dbg:
        # cv2.drawMatches uses queryIdx into img1 and trainIdx into img2. Our
        # detect helper used img_src=zoom_match, img_dst=work_w.
        try:
            mimg = cv2.drawMatches(
                cv2.cvtColor(zoom_for_draw, cv2.COLOR_GRAY2BGR), dbg["kp_z"],
                cv2.cvtColor(dbg["work_w"], cv2.COLOR_GRAY2BGR), dbg["kp_w"],
                dbg["raw_pairs"], None,
                matchColor=(0, 220, 220),       # yellow-ish for raw
                singlePointColor=(60, 60, 60),
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )
            write("matches_raw.jpg", mimg)
        except cv2.error as e:
            print(f"debug: drawMatches(raw) failed: {e}")
        metadata["n_matches"] = int(dbg.get("n_matches", len(dbg["raw_pairs"])))

    # ---- inlier matches (only if RANSAC ran successfully) ----
    if all(k in dbg for k in ("mask", "raw_pairs", "kp_z", "kp_w", "work_w")) \
            and zoom_for_draw is not None:
        mask = dbg["mask"].ravel().astype(bool)
        inlier_pairs = [m for m, keep in zip(dbg["raw_pairs"], mask) if keep]
        outlier_pairs = [m for m, keep in zip(dbg["raw_pairs"], mask) if not keep]
        try:
            mimg = cv2.drawMatches(
                cv2.cvtColor(zoom_for_draw, cv2.COLOR_GRAY2BGR), dbg["kp_z"],
                cv2.cvtColor(dbg["work_w"], cv2.COLOR_GRAY2BGR), dbg["kp_w"],
                inlier_pairs, None,
                matchColor=(0, 220, 0),  # green
                singlePointColor=(60, 60, 60),
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )
            write("matches_inliers.jpg", mimg)
        except cv2.error as e:
            print(f"debug: drawMatches(inliers) failed: {e}")
        metadata["n_inliers"] = int(dbg.get("inliers", len(inlier_pairs)))
        metadata["n_outliers"] = len(outlier_pairs)
        metadata["rms_px"] = dbg.get("rms_px")
        if "H_full" in dbg:
            metadata["H_full"] = dbg["H_full"]

    if "fit" in dbg:
        metadata["fit"] = dbg["fit"]
    if dbg.get("error"):
        metadata["error"] = dbg["error"]
    if "detector" in dbg:
        metadata["detector"] = dbg["detector"]

    with open(os.path.join(debug_dir, "metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2, default=str)
