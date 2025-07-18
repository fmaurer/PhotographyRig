import cv2
import numpy as np

LOOKUP_IMAGE = "angle_lookup.png"

# Match these to the values used in generation
HFOV_DEG = 75.0
VFOV_DEG = 60.0

def decode_angle(r, g):
    yaw = (r / 255.0) * HFOV_DEG - HFOV_DEG / 2
    pitch = (g / 255.0) * VFOV_DEG - VFOV_DEG / 2
    return yaw, pitch

def main():
    angle_img = cv2.imread(LOOKUP_IMAGE, cv2.IMREAD_COLOR)
    if angle_img is None:
        print("Failed to load angle_lookup.png")
        return

    h, w = angle_img.shape[:2]
    cx, cy = w // 2, h // 2
    test_coords = [(cx-1, cy-1),(cx, cy), (120, 320)]

    print(f"Image size: {w}x{h}")
    for x, y in test_coords:
        bgr = angle_img[y, x]
        r, g = int(bgr[2]), int(bgr[1])  # OpenCV loads BGR
        yaw, pitch = decode_angle(r, g)
        print(f"At pixel ({x}, {y}): Yaw = {yaw:.2f}°, Pitch = {pitch:.2f}°")

if __name__ == "__main__":
    main()
