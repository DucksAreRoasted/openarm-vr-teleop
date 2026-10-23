#!/usr/bin/env python3
"""Stdin/stdout server for one OpenArm arm.

Each request line is 19 floats: px py pz, then the rotation matrix row-major,
then the 7 seed joint angles. The pose is already in the robot frame.
The reply is 7 joint angles, or the word "fail".
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openarm_arm_ik import OpenArmArmIK
import numpy as np


def main():
    if len(sys.argv) != 3:
        print("usage: ik_server.py <left|right> <urdf>", file=sys.stderr)
        return 2
    side, urdf = sys.argv[1], sys.argv[2]
    ik = OpenArmArmIK(urdf, side)
    print("ready", flush=True)
    for line in sys.stdin:
        parts = line.split()
        if len(parts) != 19:
            print("fail", flush=True)
            continue
        try:
            values = [float(item) for item in parts]
        except ValueError:
            print("fail", flush=True)
            continue
        pose = np.eye(4)
        pose[:3, 3] = values[0:3]
        pose[:3, :3] = np.array(values[3:12], dtype=float).reshape(3, 3)
        seed = values[12:19]
        solved = ik.solve(pose, seed)
        if solved is None or not np.all(np.isfinite(solved)):
            print("fail", flush=True)
            continue
        print(" ".join(f"{item:.8f}" for item in solved), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
