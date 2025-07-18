import numpy as np
import cv2
import json

CALIB_JSON = "calibration_pinhole.json"
LOOKUP_IMAGE = "angle_lookup.png"

# Your FOVs
HFOV_DEG = 75.0
VFOV_DEG = 60.0

def main():
    with open(CALIB_JSON, "r") as f:
        calib = json.load(f)

    width = calib["image_width"]
    height = calib["image_height"]

    hfov_rad = np.deg2rad(HFOV_DEG)
    vfov_rad = np.deg2rad(VFOV_DEG)

    cx = width / 2.0
    cy = height / 2.0

    # Degrees per pixel
    deg_per_px_x = HFOV_DEG / width
    deg_per_px_y = VFOV_DEG / height

    # Output image with 3 channels (R=Yaw, G=Pitch)
    angle_img = np.zeros((height, width, 3), dtype=np.uint8)

    for y in range(height):
        for x in range(width):
            dx = x - cx
            dy = cy - y  # positive is "up"

            yaw_deg = dx * deg_per_px_x
            pitch_deg = dy * deg_per_px_y

            # Map from [-max_angle, max_angle] → [0, 255]
            yaw_normalized = (yaw_deg + HFOV_DEG / 2) / HFOV_DEG
            pitch_normalized = (pitch_deg + VFOV_DEG / 2) / VFOV_DEG

            # Clamp and encode to 8-bit
            r = np.clip(yaw_normalized * 255, 0, 255)
            g = np.clip(pitch_normalized * 255, 0, 255)

            #angle_img[y, x] = (int(r), int(g), 0)
            angle_img[y, x] = (0, int(g), int(r))  # B, G, R → store correctly


    cv2.imwrite(LOOKUP_IMAGE, angle_img)
    print(f"Saved angle lookup image to {LOOKUP_IMAGE}")

if __name__ == "__main__":
    main()
