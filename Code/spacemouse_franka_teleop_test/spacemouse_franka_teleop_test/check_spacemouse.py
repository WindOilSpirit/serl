#!/usr/bin/env python3
"""Print SpaceMouse samples without touching ROS or Franka."""

from __future__ import annotations

import time

import numpy as np
import pyspacemouse


def main() -> None:
    device = pyspacemouse.open()
    if device is None:
        raise SystemExit("pyspacemouse.open() failed")
    print("SpaceMouse opened. Move it or press buttons; Ctrl-C exits.")
    with np.printoptions(precision=3, suppress=True):
        try:
            while True:
                state = device.read()
                if state is not None:
                    raw = np.array(
                        [state.x, state.y, state.z, state.roll, state.pitch, state.yaw],
                        dtype=np.float64,
                    )
                    action = np.array(
                        [-state.y, state.x, state.z, -state.roll, -state.pitch, -state.yaw],
                        dtype=np.float64,
                    )
                    print(
                        "raw=[x y z roll pitch yaw]="
                        f"{raw} mapped_action=[x y z roll pitch yaw]={action} "
                        f"buttons={state.buttons}"
                    )
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            device.close()


if __name__ == "__main__":
    main()
