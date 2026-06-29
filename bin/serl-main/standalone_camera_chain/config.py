"""Standalone dual RealSense camera configuration.

Edit the serial numbers and crop regions before running the CLI.
This module intentionally has no Franka or training dependencies.
"""

# Replace these with real device serial numbers.
D455_SERIAL = ""
D435_SERIAL = ""

# Stream parameters.
WIDTH = 640
HEIGHT = 480
FPS = 30

# Crop rectangles in full-image pixel coordinates: (x1, y1, x2, y2)
D455_CROP = (0, 0, WIDTH, HEIGHT)
D435_CROP = (0, 0, WIDTH, HEIGHT)

# Output directory for saved captures.
OUTPUT_DIR = "captures"
