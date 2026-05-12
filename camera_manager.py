"""Picamera2-backed camera lifecycle for both cameras.

Replaces the rpicam-vid + ffmpeg subprocesses that MediaMTX previously
spawned via runOnInit. Owns one Picamera2 instance per camera, pushes
H.264 to MediaMTX over RTSP, and exposes live control via set_exposure /
capture_still without ever restarting the stream.

Settings are loaded from camera_settings_cam{N}.json on start_streaming;
runtime tweaks via set_exposure() go through libcamera's set_controls
(applies on the next frame) and are persisted back to JSON.
"""
import os
import threading
import time
from pathlib import Path
from typing import Optional

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
from libcamera import Transform, controls as lc_controls

import camera_settings


AWB_MODE_ENUM = {
    "auto": lc_controls.AwbModeEnum.Auto,
    "incandescent": lc_controls.AwbModeEnum.Incandescent,
    "tungsten": lc_controls.AwbModeEnum.Tungsten,
    "fluorescent": lc_controls.AwbModeEnum.Fluorescent,
    "indoor": lc_controls.AwbModeEnum.Indoor,
    "daylight": lc_controls.AwbModeEnum.Daylight,
    "cloudy": lc_controls.AwbModeEnum.Cloudy,
}

AF_MODE_ENUM = {
    "manual": lc_controls.AfModeEnum.Manual,
    "auto": lc_controls.AfModeEnum.Auto,
    "continuous": lc_controls.AfModeEnum.Continuous,
}


class _CameraStream:
    """One camera's streaming state."""

    def __init__(self, cam_idx: int, rtsp_host: str, bitrate: int):
        self.cam_idx = cam_idx
        self.rtsp_host = rtsp_host
        self.bitrate = bitrate
        self.picam2: Optional[Picamera2] = None
        self.encoder: Optional[H264Encoder] = None
        self.output: Optional[FfmpegOutput] = None
        self.lock = threading.Lock()  # serializes (re)configuration and capture

    @property
    def rtsp_url(self) -> str:
        return f"rtsp://{self.rtsp_host}:8554/cam{self.cam_idx}"

    def _build_controls(self, settings: dict) -> dict:
        controls = {
            "ExposureTime": int(settings["ExposureTime"]),
            "AnalogueGain": float(settings["AnalogueGain"]),
            "AeEnable": bool(settings["AeEnable"]),
            "FrameRate": 30.0,
        }
        if settings.get("AwbEnable") is not None:
            controls["AwbEnable"] = bool(settings["AwbEnable"])
        awb_mode = settings.get("AwbMode")
        if awb_mode and awb_mode in AWB_MODE_ENUM:
            controls["AwbMode"] = AWB_MODE_ENUM[awb_mode]
        af_mode = settings.get("AfMode")
        if af_mode and af_mode in AF_MODE_ENUM:
            controls["AfMode"] = AF_MODE_ENUM[af_mode]
        if settings.get("LensPosition") is not None:
            # Camera_controls populates after Picamera2(...) — checked in start.
            controls["LensPosition"] = float(settings["LensPosition"])
        return controls

    def start(self) -> None:
        with self.lock:
            if self.picam2 is not None:
                return
            settings = camera_settings.load(self.cam_idx)
            self.picam2 = Picamera2(camera_num=self.cam_idx)

            initial_controls = self._build_controls(settings)
            # Drop LensPosition if this camera doesn't expose it (e.g. HQ Cam fixed lens).
            if "LensPosition" in initial_controls and "LensPosition" not in self.picam2.camera_controls:
                initial_controls.pop("LensPosition")
                initial_controls.pop("AfMode", None)

            transform = Transform(
                hflip=int(settings["Transform"]["hflip"]),
                vflip=int(settings["Transform"]["vflip"]),
            )
            video_config = self.picam2.create_video_configuration(
                main={"size": tuple(settings["StreamSize"]), "format": "YUV420"},
                controls=initial_controls,
                transform=transform,
                queue=True,
            )
            self.picam2.configure(video_config)

            try:
                self.encoder = H264Encoder(bitrate=self.bitrate, profile="baseline")
            except TypeError:
                self.encoder = H264Encoder(bitrate=self.bitrate)
            self.output = FfmpegOutput(
                f"-f rtsp -rtsp_transport tcp {self.rtsp_url}",
                audio=False,
            )
            self.picam2.start_recording(self.encoder, self.output)
            print(f"camera_manager: cam{self.cam_idx} streaming -> {self.rtsp_url} "
                  f"({settings['StreamSize'][0]}x{settings['StreamSize'][1]} "
                  f"@ {self.bitrate/1e6:.1f} Mbit/s)")

    def stop(self) -> None:
        with self.lock:
            if self.picam2 is None:
                return
            try:
                self.picam2.stop_recording()
            except Exception as e:
                print(f"camera_manager: cam{self.cam_idx} stop_recording: {e!r}")
            try:
                self.picam2.close()
            except Exception as e:
                print(f"camera_manager: cam{self.cam_idx} close: {e!r}")
            self.picam2 = None
            self.encoder = None
            self.output = None

    def set_exposure(self, *, shutter_us: Optional[int] = None,
                     gain: Optional[float] = None,
                     ae_enable: Optional[bool] = None,
                     persist: bool = True) -> dict:
        """Apply exposure changes live (no restart) and optionally persist."""
        with self.lock:
            if self.picam2 is None:
                raise RuntimeError(f"cam{self.cam_idx} is not streaming")
            controls = {}
            changes = {}
            if shutter_us is not None:
                controls["ExposureTime"] = int(shutter_us)
                changes["ExposureTime"] = int(shutter_us)
                # Setting manual shutter implies AE off unless caller said otherwise.
                if ae_enable is None:
                    controls["AeEnable"] = False
                    changes["AeEnable"] = False
            if gain is not None:
                controls["AnalogueGain"] = float(gain)
                changes["AnalogueGain"] = float(gain)
                if ae_enable is None:
                    controls["AeEnable"] = False
                    changes["AeEnable"] = False
            if ae_enable is not None:
                controls["AeEnable"] = bool(ae_enable)
                changes["AeEnable"] = bool(ae_enable)
            if not controls:
                return camera_settings.load(self.cam_idx)
            self.picam2.set_controls(controls)
            if persist:
                return camera_settings.update(self.cam_idx, **changes)
            merged = camera_settings.load(self.cam_idx)
            merged.update(changes)
            return merged

    def set_ae_compensation(self, ev_stops: float, persist: bool = True) -> dict:
        """Set ExposureValue (AE bias, stops). Only meaningful with AeEnable=True;
        enabling AE if it isn't already."""
        with self.lock:
            if self.picam2 is None:
                raise RuntimeError(f"cam{self.cam_idx} is not streaming")
            self.picam2.set_controls({
                "AeEnable": True,
                "ExposureValue": float(ev_stops),
            })
            if persist:
                return camera_settings.update(self.cam_idx,
                                              AeEnable=True,
                                              ExposureValue=float(ev_stops))
            merged = camera_settings.load(self.cam_idx)
            merged["AeEnable"] = True
            merged["ExposureValue"] = float(ev_stops)
            return merged

    def set_awb(self, *, enable: Optional[bool] = None,
                mode: Optional[str] = None, persist: bool = True) -> dict:
        with self.lock:
            if self.picam2 is None:
                raise RuntimeError(f"cam{self.cam_idx} is not streaming")
            controls = {}
            changes = {}
            if enable is not None:
                controls["AwbEnable"] = bool(enable)
                changes["AwbEnable"] = bool(enable)
            if mode is not None and mode in AWB_MODE_ENUM:
                controls["AwbMode"] = AWB_MODE_ENUM[mode]
                changes["AwbMode"] = mode
            if not controls:
                return camera_settings.load(self.cam_idx)
            self.picam2.set_controls(controls)
            if persist:
                return camera_settings.update(self.cam_idx, **changes)
            merged = camera_settings.load(self.cam_idx)
            merged.update(changes)
            return merged

    def set_lens_position(self, position: float, persist: bool = True) -> dict:
        with self.lock:
            if self.picam2 is None:
                raise RuntimeError(f"cam{self.cam_idx} is not streaming")
            if "LensPosition" not in self.picam2.camera_controls:
                raise RuntimeError(f"cam{self.cam_idx} has no LensPosition control")
            self.picam2.set_controls({
                "AfMode": lc_controls.AfModeEnum.Manual,
                "LensPosition": float(position),
            })
            if persist:
                return camera_settings.update(self.cam_idx,
                                              LensPosition=float(position),
                                              AfMode="manual")
            merged = camera_settings.load(self.cam_idx)
            merged["LensPosition"] = float(position)
            merged["AfMode"] = "manual"
            return merged

    def capture_still(self, path: str) -> str:
        """Capture a JPEG without interrupting the stream. Returns the path."""
        with self.lock:
            if self.picam2 is None:
                raise RuntimeError(f"cam{self.cam_idx} is not streaming")
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            request = self.picam2.capture_request()
            try:
                request.save("main", path)
            finally:
                request.release()
            return path

    def camera_controls_snapshot(self) -> dict:
        with self.lock:
            if self.picam2 is None:
                return {}
            return dict(self.picam2.camera_controls)


class CameraManager:
    """Top-level: owns both cameras' streams.

    Lifecycle:
        cm = CameraManager()
        cm.start_all()      # starts streaming for every camera in CAMERA_INDICES
        ...
        cm.set_exposure(0, shutter_us=50000, gain=2.0)
        cm.capture_still(0, "./captures/foo.jpg")
        ...
        cm.stop_all()
    """

    CAMERA_INDICES = (0, 1)

    def __init__(self, rtsp_host: str = "localhost", bitrate: int = 1_000_000):
        self.streams = {
            idx: _CameraStream(idx, rtsp_host, bitrate) for idx in self.CAMERA_INDICES
        }

    def start_all(self) -> None:
        for idx in self.CAMERA_INDICES:
            try:
                self.streams[idx].start()
            except Exception as e:
                print(f"camera_manager: failed to start cam{idx}: {e!r}")

    def stop_all(self) -> None:
        for idx in self.CAMERA_INDICES:
            try:
                self.streams[idx].stop()
            except Exception as e:
                print(f"camera_manager: failed to stop cam{idx}: {e!r}")

    def _stream(self, cam_idx: int) -> _CameraStream:
        if cam_idx not in self.streams:
            raise KeyError(f"unknown cam_idx {cam_idx}")
        return self.streams[cam_idx]

    def set_exposure(self, cam_idx: int, *, shutter_us: Optional[int] = None,
                     gain: Optional[float] = None,
                     ae_enable: Optional[bool] = None,
                     persist: bool = True) -> dict:
        return self._stream(cam_idx).set_exposure(
            shutter_us=shutter_us, gain=gain, ae_enable=ae_enable, persist=persist
        )

    def set_awb(self, cam_idx: int, *, enable: Optional[bool] = None,
                mode: Optional[str] = None, persist: bool = True) -> dict:
        return self._stream(cam_idx).set_awb(enable=enable, mode=mode, persist=persist)

    def set_ae_compensation(self, cam_idx: int, ev_stops: float,
                            persist: bool = True) -> dict:
        return self._stream(cam_idx).set_ae_compensation(ev_stops, persist=persist)

    def set_lens_position(self, cam_idx: int, position: float,
                          persist: bool = True) -> dict:
        return self._stream(cam_idx).set_lens_position(position, persist=persist)

    def capture_still(self, cam_idx: int, path: str) -> str:
        return self._stream(cam_idx).capture_still(path)

    def camera_controls_snapshot(self, cam_idx: int) -> dict:
        return self._stream(cam_idx).camera_controls_snapshot()
