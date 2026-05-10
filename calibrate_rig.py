#!/usr/bin/env python3
"""
Calibrate the IMU for the photography rig.

Place the camera lens assembly in the "level" position before running.
This script reads the IMU, averages multiple samples, and saves offsets
to calibration.json so that websocket_server.py can display corrected values.

Target orientation when level: pitch=90, roll=0, yaw=0
"""

import time
import math
import json
import os
import datetime

import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.json')
NUM_SAMPLES = 50
SAMPLE_INTERVAL = 0.1  # seconds between samples


def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main():
    print("=== IMU Calibration ===")
    print("Ensure the lens assembly is level before continuing.")
    print()

    # Initialize IMU
    print("Initializing IMU...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    bno = BNO08X_I2C(i2c)
    time.sleep(1)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    print("IMU initialized.")

    # Discard first few readings to let the sensor settle
    print("Warming up sensor (discarding initial readings)...")
    for _ in range(10):
        try:
            bno.quaternion
        except Exception:
            pass
        time.sleep(0.1)

    # Collect samples
    print(f"Collecting {NUM_SAMPLES} samples...")
    pitch_samples = []
    roll_samples = []
    yaw_samples = []

    for i in range(NUM_SAMPLES):
        try:
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            rx, ry, rz = quaternion_to_euler(quat_real, quat_i, quat_j, quat_k)

            pitch_deg = math.degrees(rx)
            roll_deg = math.degrees(ry)
            yaw_deg = math.degrees(rz)

            pitch_samples.append(pitch_deg)
            roll_samples.append(roll_deg)
            yaw_samples.append(yaw_deg)

            if (i + 1) % 10 == 0:
                print(f"  {i + 1}/{NUM_SAMPLES} samples collected...")

        except Exception as e:
            print(f"  Warning: failed to read sample {i + 1}: {e}")

        time.sleep(SAMPLE_INTERVAL)

    if not pitch_samples:
        print("ERROR: No valid samples collected. Aborting.")
        return

    # Average the samples
    avg_pitch = sum(pitch_samples) / len(pitch_samples)
    avg_roll = sum(roll_samples) / len(roll_samples)
    avg_yaw = sum(yaw_samples) / len(yaw_samples)

    print()
    print(f"Raw averages ({len(pitch_samples)} samples):")
    print(f"  Pitch: {avg_pitch:.3f}")
    print(f"  Roll:  {avg_roll:.3f}")
    print(f"  Yaw:   {avg_yaw:.3f}")

    # Compute offsets so that level reads as (90, 0, 0)
    # calibrated = raw - offset
    # At level: 90 = avg_pitch - pitch_offset  =>  pitch_offset = avg_pitch - 90
    # At level:  0 = avg_roll  - roll_offset   =>  roll_offset  = avg_roll
    pitch_offset = avg_pitch - 90.0
    roll_offset = avg_roll

    print()
    print(f"Computed offsets:")
    print(f"  pitch_offset: {pitch_offset:.3f}  (raw {avg_pitch:.3f} -> 90.0)")
    print(f"  roll_offset:  {roll_offset:.3f}  (raw {avg_roll:.3f} -> 0.0)")

    # Build calibration data
    calibration = {
        "pitch_offset": round(pitch_offset, 4),
        "roll_offset": round(roll_offset, 4),
        "raw_pitch_at_level": round(avg_pitch, 4),
        "raw_roll_at_level": round(avg_roll, 4),
        "raw_yaw_at_level": round(avg_yaw, 4),
        "num_samples": len(pitch_samples),
        "calibrated_at": datetime.datetime.now().isoformat(),
    }

    # Save
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(calibration, f, indent=2)

    print()
    print(f"Calibration saved to {CALIBRATION_FILE}")
    print()
    print("After calibration, websocket_server.py will display:")
    print("  Pitch: 90.0  |  Roll: 0.0  |  Yaw: (uncorrected)")


if __name__ == '__main__':
    main()
