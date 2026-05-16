"""Picamera2-backed camera lifecycle for both cameras.

Replaces the rpicam-vid + ffmpeg subprocesses that MediaMTX previously
spawned via runOnInit. Owns one Picamera2 instance per camera, pushes
H.264 to MediaMTX over RTSP, and exposes live control via set_exposure /
capture_still without ever restarting the stream.

Settings are loaded from camera_settings_cam{N}.json on start_streaming;
runtime tweaks via set_exposure() go through libcamera's set_controls
(applies on the next frame) and are persisted back to JSON.
"""
import datetime
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
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

    def capture_array(self, *, color: str = "gray") -> np.ndarray:
        """Capture a single frame as a numpy array without interrupting the stream.

        color = "gray" returns a (H, W) uint8 grayscale array — cheapest path,
        suitable for feature detection.
        color = "bgr"  returns a (H, W, 3) uint8 BGR array.

        The stream is configured for YUV420 (see start()), so we release the
        request as quickly as possible to keep the H.264 encoder fed, then do
        the colour-space conversion outside the lock.
        """
        if color not in ("gray", "bgr"):
            raise ValueError(f"color must be 'gray' or 'bgr', got {color!r}")
        with self.lock:
            if self.picam2 is None:
                raise RuntimeError(f"cam{self.cam_idx} is not streaming")
            request = self.picam2.capture_request()
            try:
                # YUV420 layout: (H*3/2, W) uint8. make_array() copies the
                # buffer so it's safe to release the request immediately.
                yuv = request.make_array("main")
            finally:
                request.release()
        if color == "gray":
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2GRAY_I420)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

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

    def __init__(self, rtsp_host: str = "localhost", bitrate: int = 1_000_000,
                 recording_dir: str = "recordings"):
        self.rtsp_host = rtsp_host
        self.streams = {
            idx: _CameraStream(idx, rtsp_host, bitrate) for idx in self.CAMERA_INDICES
        }
        self.recording_dir = recording_dir
        self.recording_procs: dict = {}
        self.recording_lock = threading.Lock()

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

    def capture_array(self, cam_idx: int, *, color: str = "gray") -> np.ndarray:
        return self._stream(cam_idx).capture_array(color=color)

    def camera_controls_snapshot(self, cam_idx: int) -> dict:
        return self._stream(cam_idx).camera_controls_snapshot()

    def is_recording(self) -> bool:
        with self.recording_lock:
            return bool(self.recording_procs)

    def start_recording(self) -> dict:
        """Start an ffmpeg subprocess per camera that remuxes the live RTSP
        feed into MP4 with stream copy (no re-encode). The Picamera2 H.264
        encoder keeps running untouched."""
        with self.recording_lock:
            if self.recording_procs:
                return {
                    "status": "already_recording",
                    "files": [self._recording_path(idx, self._current_ts)
                              for idx in self.recording_procs],
                    "timestamp": self._current_ts,
                }

            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(self.recording_dir, exist_ok=True)
            files = []
            for cam_idx in self.CAMERA_INDICES:
                output = os.path.join(self.recording_dir, f"cam{cam_idx}_{ts}.mp4")
                cmd = [
                    "ffmpeg", "-y",
                    "-rtsp_transport", "tcp",
                    "-i", f"rtsp://{self.rtsp_host}:8554/cam{cam_idx}",
                    "-c", "copy",
                    "-f", "mp4",
                    output,
                ]
                proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.recording_procs[cam_idx] = proc
                files.append(output)
                print(f"camera_manager: cam{cam_idx} recording -> {output}")

            self._current_ts = ts
            return {"status": "recording", "files": files, "timestamp": ts}

    def stop_recording(self) -> dict:
        """Gracefully finalise every running recording.

        ffmpeg watches stdin for 'q' and finalises the MP4 (writes moov atom)
        on receipt — far safer than SIGTERM, which can leave the file
        unplayable. We fall back to terminate/kill if the graceful path
        stalls."""
        with self.recording_lock:
            if not self.recording_procs:
                return {"status": "not_recording"}

            ts = getattr(self, "_current_ts", None)
            files = []
            for cam_idx, proc in self.recording_procs.items():
                output = self._recording_path(cam_idx, ts)
                files.append(output)
                try:
                    if proc.stdin and not proc.stdin.closed:
                        proc.stdin.write(b"q")
                        proc.stdin.flush()
                        proc.stdin.close()
                except (BrokenPipeError, OSError) as e:
                    print(f"camera_manager: cam{cam_idx} stdin write: {e!r}")
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(f"camera_manager: cam{cam_idx} ffmpeg didn't quit "
                          f"gracefully, terminating")
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                print(f"camera_manager: cam{cam_idx} recording stopped -> {output}")

            self.recording_procs.clear()
            self._current_ts = None
            return {"status": "stopped", "files": files, "timestamp": ts}

    def _recording_path(self, cam_idx: int, ts: Optional[str]) -> str:
        if ts is None:
            return ""
        return os.path.join(self.recording_dir, f"cam{cam_idx}_{ts}.mp4")
