import time
import os
from picamera2 import Picamera2
from datetime import datetime

OUTPUT_DIR = "calibration_images"
CAMERA_INDEX = 1
NUM_IMAGES = 20
CAPTURE_INTERVAL_SEC = 0.5

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    picam2 = Picamera2(camera_num=CAMERA_INDEX)
    config = picam2.create_still_configuration(main={"size": (1280, 720)})
    picam2.configure(config)
    picam2.start()
    print("Warming up camera...")

    time.sleep(2)
    print("Capturing images. Move the checkerboard between each shot.")

    for i in range(NUM_IMAGES):
        filename = f"{OUTPUT_DIR}/calib_{i:02d}.jpg"
        picam2.capture_file(filename)
        print(f"Captured {filename}")
        time.sleep(CAPTURE_INTERVAL_SEC)

    picam2.stop()
    print("Done.")

if __name__ == "__main__":
    main()
