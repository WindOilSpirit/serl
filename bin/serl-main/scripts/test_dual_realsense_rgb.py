"""
Test script to open two RealSense cameras (D455 and D435) and display/save RGB full and crop images.

Usage:
    python scripts/test_dual_realsense_rgb.py

Keys:
    s - save current full+crop images for both cameras
    q / ESC - quit

This script reuses `franka_env.camera.rs_capture.RSCapture` and
`franka_env.camera.video_capture.VideoCapture` so it coexists with existing env camera init.
"""
import os
import time
from datetime import datetime
import cv2
import numpy as np
import pyrealsense2 as rs

from franka_env.camera.rs_capture import RSCapture
from franka_env.camera.video_capture import VideoCapture

from experiments.shape_insertion import config


def list_serials():
    return [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]


def safe_crop(img, crop):
    h, w = img.shape[:2]
    x1, y1, x2, y2 = crop
    x1 = max(0, int(x1))
    y1 = max(0, int(y1))
    x2 = min(w, int(x2))
    y2 = min(h, int(y2))
    if x1 >= x2 or y1 >= y2:
        raise ValueError(f"Invalid crop {crop} for image shape {(w,h)}")
    return img[y1:y2, x1:x2]


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def main():
    # Validate serial numbers
    available = list_serials()
    if config.D455_SERIAL == "" or config.D435_SERIAL == "":
        print("Please set D455_SERIAL and D435_SERIAL in experiments/shape_insertion/config.py")
        return

    if config.D455_SERIAL not in available:
        print(f"D455 serial {config.D455_SERIAL} not found. Available: {available}")
        return
    if config.D435_SERIAL not in available:
        print(f"D435 serial {config.D435_SERIAL} not found. Available: {available}")
        return

    # Initialize captures
    try:
        cap_d455 = VideoCapture(
            RSCapture(name="d455", serial_number=config.D455_SERIAL, dim=(config.WIDTH, config.HEIGHT), fps=config.FPS, depth=False)
        )
        cap_d435 = VideoCapture(
            RSCapture(name="d435", serial_number=config.D435_SERIAL, dim=(config.WIDTH, config.HEIGHT), fps=config.FPS, depth=False)
        )
    except AssertionError as e:
        print(f"Camera init failed: {e}")
        return
    except Exception as e:
        print(f"Unexpected camera init error: {e}")
        return

    print("Cameras initialized. Press 's' to save images, 'q' to quit.")

    save_counter = 0
    ensure_dir(config.OUTPUT_DIR)

    try:
        while True:
            try:
                img_d455 = cap_d455.read()
                img_d435 = cap_d435.read()
            except Exception as e:
                print(f"Error reading frames: {e}")
                break

            # show full images
            try:
                crop_d455 = safe_crop(img_d455, config.D455_CROP)
            except Exception as e:
                print(f"D455 crop error: {e}")
                crop_d455 = img_d455

            try:
                crop_d435 = safe_crop(img_d435, config.D435_CROP)
            except Exception as e:
                print(f"D435 crop error: {e}")
                crop_d435 = img_d435

            # show windows
            cv2.imshow('d455_full', img_d455)
            cv2.imshow('d455_crop', crop_d455)
            cv2.imshow('d435_full', img_d435)
            cv2.imshow('d435_crop', crop_d435)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                outdir = os.path.join(config.OUTPUT_DIR, f"{ts}_{save_counter}")
                ensure_dir(outdir)
                # Save full and crop images
                cv2.imwrite(os.path.join(outdir, 'd455_full.png'), img_d455)
                cv2.imwrite(os.path.join(outdir, 'd455_crop.png'), crop_d455)
                cv2.imwrite(os.path.join(outdir, 'd435_full.png'), img_d435)
                cv2.imwrite(os.path.join(outdir, 'd435_crop.png'), crop_d435)
                print(f"Saved images to {outdir}")
                save_counter += 1
            elif key == ord('q') or key == 27:
                break

    finally:
        try:
            cap_d455.close()
        except Exception:
            pass
        try:
            cap_d435.close()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
