import json
import cv2
import numpy as np
from picamera2 import Picamera2
import time

CALIB_JSON = "calibration_pinhole.json"
CAMERA_INDEX = 1

def load_calibration(path):
    with open(path, "r") as f:
        data = json.load(f)
    K = np.array(data["K"])
    D = np.array(data["D"])
    dim = (data["image_width"], data["image_height"])
    return K, D, dim

def main():
    K, D, dim = load_calibration(CALIB_JSON)
    print("Loaded calibration parameters.")

    picam2 = Picamera2(camera_num=CAMERA_INDEX)
    config = picam2.create_preview_configuration(main={"size": dim})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, dim, 1, dim)
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, new_K, dim, cv2.CV_16SC2)
    print("Undistortion map created.")

    print("Press ESC to quit.")
    while True:
        frame = picam2.capture_array()
        undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

        preview = np.hstack((frame, undistorted))
        cv2.imshow("Original (left) vs Undistorted (right)", preview)

        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break

    cv2.destroyAllWindows()
    picam2.stop()

if __name__ == "__main__":
    main()
