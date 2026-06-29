# Standalone Camera Chain

This folder is intentionally isolated from Franka control, training, reward models, replay buffers, and task wrappers.

## What it does

- Lists connected Intel RealSense devices
- Opens two cameras independently using serial numbers
- Captures RGB frames from each camera
- Applies per-camera crop regions
- Saves full and cropped images to disk
- Provides a small interactive command loop for testing camera interfaces

## Files

- `config.py`: serial numbers, crop regions, and stream parameters
- `dual_realsense_cli.py`: standalone interactive capture script

## How to use

1. Fill in `D455_SERIAL` and `D435_SERIAL` in `config.py`.
2. Run:

```bash
python standalone_camera_chain/dual_realsense_cli.py
```

3. Use the commands printed by the script:

- `list`
- `capture d455`
- `capture d435`
- `capture both`
- `preview`
- `quit`

## Output

Captures are saved under `captures/<timestamp>/` as:

- `d455_full.png`
- `d455_crop.png`
- `d435_full.png`
- `d435_crop.png`

## Notes

- The script only depends on `pyrealsense2`, `numpy`, and `opencv-python`.
- It does not import any `franka_*` or `serl_launcher` modules.
