import cv2
import numpy as np
import json

CALIB_JSON = "calibration_pinhole.json"
INPUT_IMAGE = "calibration_images/calib_00.jpg"
OUTPUT_IMAGE = "undistorted.jpg"

def load_calibration(path):
    with open(path, "r") as f:
        data = json.load(f)
    K = np.array(data["K"])
    D = np.array(data["D"])
    dim = (data["image_width"], data["image_height"])
    return K, D, dim

def undistort_pinhole(input_path, output_path, K, D, dim):
    img = cv2.imread(input_path)
    if img.shape[1::-1] != dim:
        print(f"Resizing image from {img.shape[1::-1]} to {dim}")
        img = cv2.resize(img, dim)

    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, dim, 1, dim)
    undistorted = cv2.undistort(img, K, D, None, new_K)

    x, y, w, h = roi
    undistorted_cropped = undistorted[y:y+h, x:x+w]
    cv2.imwrite(output_path, undistorted_cropped)
    print(f"Saved undistorted image to {output_path}")

if __name__ == "__main__":
    K, D, dim = load_calibration(CALIB_JSON)
    undistort_pinhole(INPUT_IMAGE, OUTPUT_IMAGE, K, D, dim)
