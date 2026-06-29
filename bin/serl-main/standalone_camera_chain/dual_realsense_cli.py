#!/usr/bin/env python3
"""Standalone dual RealSense RGB CLI.

Features:
- list connected RealSense devices
- open D455 and D435 independently
- capture one camera or both cameras on demand
- preview current frames
- save full and cropped images

This script is intentionally isolated from Franka control, reward logic,
training, replay buffers, and task wrappers.
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs

import config


Crop = Tuple[int, int, int, int]


@dataclass
class CameraSpec:
    name: str
    serial: str
    crop: Crop


class RSCapture:
    def __init__(self, name: str, serial_number: str, dim=(640, 480), fps=30):
        self.name = name
        self.serial_number = serial_number
        self.dim = dim
        self.fps = fps
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_device(self.serial_number)
        self.cfg.enable_stream(rs.stream.color, dim[0], dim[1], rs.format.bgr8, fps)
        self.profile = self.pipe.start(self.cfg)

    @staticmethod
    def list_serials() -> list[str]:
        return [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]

    def read(self) -> np.ndarray:
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame.is_video_frame():
            raise RuntimeError(f"{self.name}: invalid color frame")
        return np.asarray(color_frame.get_data())

    def close(self) -> None:
        self.pipe.stop()
        self.cfg.disable_all_streams()


class VideoCapture:
    def __init__(self, cap: RSCapture):
        self.cap = cap

    def read(self) -> np.ndarray:
        return self.cap.read()

    def close(self) -> None:
        self.cap.close()


def safe_crop(img: np.ndarray, crop: Crop) -> np.ndarray:
    h, w = img.shape[:2]
    x1, y1, x2, y2 = crop
    x1 = max(0, int(x1))
    y1 = max(0, int(y1))
    x2 = min(w, int(x2))
    y2 = min(h, int(y2))
    if x1 >= x2 or y1 >= y2:
        raise ValueError(f"Invalid crop {crop} for image shape {(w, h)}")
    return img[y1:y2, x1:x2]


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


class DualCameraChain:
    def __init__(self, d455_serial: str, d435_serial: str, width: int, height: int, fps: int):
        self.specs: Dict[str, CameraSpec] = {
            "d455": CameraSpec("d455", d455_serial, config.D455_CROP),
            "d435": CameraSpec("d435", d435_serial, config.D435_CROP),
        }
        self.caps = {
            name: VideoCapture(RSCapture(spec.name, spec.serial, dim=(width, height), fps=fps))
            for name, spec in self.specs.items()
        }

    def read(self) -> Dict[str, np.ndarray]:
        return {name: cap.read() for name, cap in self.caps.items()}

    def capture(self) -> Dict[str, Dict[str, np.ndarray]]:
        frames = self.read()
        result: Dict[str, Dict[str, np.ndarray]] = {}
        for name, frame in frames.items():
            crop = safe_crop(frame, self.specs[name].crop)
            result[name] = {"full": frame, "crop": crop}
        return result

    def close(self) -> None:
        for cap in self.caps.values():
            try:
                cap.close()
            except Exception:
                pass


def print_available_devices() -> None:
    serials = RSCapture.list_serials()
    print("Available RealSense serial numbers:")
    for serial in serials:
        print(f"  - {serial}")


def save_capture(output_dir: Path, captures: Dict[str, Dict[str, np.ndarray]]) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = output_dir / stamp
    ensure_dir(folder)
    for name, variants in captures.items():
        cv2.imwrite(str(folder / f"{name}_full.png"), variants["full"])
        cv2.imwrite(str(folder / f"{name}_crop.png"), variants["crop"])
    return folder


def preview_loop(chain: DualCameraChain, output_dir: Path) -> None:
    print("Preview controls: s=save current frames, q=quit preview")
    while True:
        captures = chain.capture()
        view = np.concatenate([captures[name]["crop"] for name in ("d455", "d435")], axis=0)
        cv2.imshow("dual_realsense_preview", view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            folder = save_capture(output_dir, captures)
            print(f"Saved preview capture to {folder}")
        elif key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def command_loop(chain: DualCameraChain, output_dir: Path) -> None:
    print("Commands: list, capture d455, capture d435, capture both, preview, quit")
    while True:
        try:
            command = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if command in {"quit", "exit", "q"}:
            break
        if command == "list":
            print_available_devices()
            continue
        if command == "preview":
            preview_loop(chain, output_dir)
            continue
        if command.startswith("capture"):
            parts = command.split()
            if len(parts) != 2 or parts[1] not in {"d455", "d435", "both"}:
                print("Usage: capture d455 | capture d435 | capture both")
                continue
            captures = chain.capture()
            if parts[1] == "d455":
                captures = {"d455": captures["d455"]}
            elif parts[1] == "d435":
                captures = {"d435": captures["d435"]}
            folder = save_capture(output_dir, captures)
            print(f"Saved capture to {folder}")
            continue

        print("Unknown command. Available: list, capture d455, capture d435, capture both, preview, quit")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone dual RealSense RGB chain")
    parser.add_argument("--output-dir", default=config.OUTPUT_DIR, help="Directory for saved images")
    parser.add_argument("--width", type=int, default=config.WIDTH, help="Stream width")
    parser.add_argument("--height", type=int, default=config.HEIGHT, help="Stream height")
    parser.add_argument("--fps", type=int, default=config.FPS, help="Stream fps")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not config.D455_SERIAL or not config.D435_SERIAL:
        raise SystemExit("Please set D455_SERIAL and D435_SERIAL in config.py")

    output_dir = Path(args.output_dir)
    ensure_dir(output_dir)

    print_available_devices()
    available = RSCapture.list_serials()
    if config.D455_SERIAL not in available:
        raise SystemExit(f"D455 serial {config.D455_SERIAL} not found")
    if config.D435_SERIAL not in available:
        raise SystemExit(f"D435 serial {config.D435_SERIAL} not found")

    chain = DualCameraChain(
        d455_serial=config.D455_SERIAL,
        d435_serial=config.D435_SERIAL,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    try:
        command_loop(chain, output_dir)
    finally:
        chain.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
