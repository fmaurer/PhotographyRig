import cv2
import numpy as np
import json
import os
from glob import glob

CHECKERBOARD = (9, 6)  # number of inner corners (NOT squares!)
IMAGE_DIR = "calibration_images"
CALIB_JSON = "calibration_pinhole.json"

def main():
    img_paths = glob(os.path.join(IMAGE_DIR, "*.jpg"))
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    objpoints = []
    imgpoints = []

    for path in img_paths:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            imgpoints.append(corners2)
            objpoints.append(objp)
            print(f"Found corners in {path}")
        else:
            print(f"Skipped {path}, no corners found.")

    N_OK = len(objpoints)
    print(f"Found corners in {N_OK} images")

    if N_OK < 5:
        print("Not enough valid images for calibration.")
        return

    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print(f"RMS re-projection error: {ret}")

    calibration_data = {
        "K": K.tolist(),
        "D": D.tolist(),
        "image_width": gray.shape[1],
        "image_height": gray.shape[0]
    }

    with open(CALIB_JSON, "w") as f:
        json.dump(calibration_data, f, indent=2)

    print(f"Saved calibration to {CALIB_JSON}")

if __name__ == "__main__":
    main()
