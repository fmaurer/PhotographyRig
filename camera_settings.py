"""Per-camera settings: load/save/defaults for camera_settings_cam{N}.json.

Single source of truth for exposure, white balance, transforms, and lens
position. Used by both the live streamer (CameraManager) and the still
capture path. Replaces the ad-hoc JSON shape that fastPicture.py used to
write.

Keep the schema additive — missing keys fall back to defaults so older
JSON files keep working.
"""
import json
import os
from typing import Any, Optional

SETTINGS_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULTS = {
    # cam0 — Raspberry Pi HQ Camera (IMX477)
    0: {
        "ExposureTime": 100_000,   # microseconds
        "AnalogueGain": 1.0,
        "AeEnable": False,         # default manual; UI can flip to True
        "AwbEnable": True,
        "AwbMode": "auto",         # "auto"|"incandescent"|"tungsten"|"fluorescent"|"indoor"|"daylight"|"cloudy"
        "Transform": {"hflip": False, "vflip": False},
        "LensPosition": None,      # IMX477 has fixed lens; None = leave alone
        "AfMode": None,            # None|"manual"|"auto"|"continuous"
        "StreamSize": [1280, 720],
        "StillSize": [4056, 3040],
    },
    # cam1 — Arducam 64MP (Pivariety). Matches current rpicam-vid args:
    #   --shutter 1000 --hflip --vflip --lens-position 6 --autofocus-mode manual
    1: {
        "ExposureTime": 1_000,
        "AnalogueGain": 1.0,
        "AeEnable": False,
        "AwbEnable": True,
        "AwbMode": "auto",
        "Transform": {"hflip": True, "vflip": True},
        "LensPosition": 6.0,
        "AfMode": "manual",
        "StreamSize": [1280, 720],
        "StillSize": [9152, 6944],  # Arducam 64MP native
    },
}


def _path(cam_idx: int) -> str:
    return os.path.join(SETTINGS_DIR, f"camera_settings_cam{cam_idx}.json")


def load(cam_idx: int) -> dict:
    """Return per-cam settings, merging file values over defaults."""
    settings = dict(DEFAULTS[cam_idx])
    path = _path(cam_idx)
    if os.path.exists(path):
        try:
            with open(path) as f:
                on_disk = json.load(f)
            # Shallow merge — file wins for keys it specifies; missing keys use defaults.
            for k, v in on_disk.items():
                settings[k] = v
        except (json.JSONDecodeError, OSError) as e:
            print(f"camera_settings.load(cam{cam_idx}): {e!r} — using defaults")
    return settings


def save(cam_idx: int, settings: dict) -> None:
    """Persist settings. Caller is responsible for the merge if doing partial updates."""
    path = _path(cam_idx)
    with open(path, "w") as f:
        json.dump(settings, f, indent=2)


def update(cam_idx: int, **changes: Any) -> dict:
    """Load -> apply changes -> save. Returns the merged dict."""
    settings = load(cam_idx)
    settings.update(changes)
    save(cam_idx, settings)
    return settings


def get_field(cam_idx: int, key: str, default: Optional[Any] = None) -> Any:
    return load(cam_idx).get(key, default)
