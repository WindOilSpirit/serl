"""
Configuration for shape_insertion camera test.

Edit the serial numbers and crop regions before running the test script.
"""

# Fill these with values discovered from `rs-enumerate-devices`
D455_SERIAL = "313522302591"  # e.g. "130322274175"
D435_SERIAL = "349622074877"  # e.g. "127122270572"

# Image stream params
WIDTH = 640
HEIGHT = 480
FPS = 30

# Crop tuples: (x1, y1, x2, y2) in pixel coordinates on full image
D455_CROP = (0, 0, WIDTH, HEIGHT)
D435_CROP = (0, 0, WIDTH, HEIGHT)

# Observation keys to be used later when adding to env observation dict
OBS_KEYS = {
    "d455": "d455_rgb",
    "d435": "d435_rgb",
}

# Output data folder for saved debug images
OUTPUT_DIR = "data/camera_debug"
