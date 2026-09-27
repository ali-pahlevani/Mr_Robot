#!/usr/bin/env python3
"""Turn a raw slam_toolbox save into maps/kitchen.{pgm,yaml}.

    ros2 run mrrobot_navigation finish_map.py <slam_map.yaml> <out_dir/kitchen>

Two edits, both explained in maps/README.md:

  1. Shift the origin by the robot's spawn point (0.15, 0.20). slam_toolbox's
     map frame starts where the robot started, with world-aligned axes
     (the EKF takes heading from the IMU), so adding the spawn offset makes
     the map frame coincide with the Webots world frame -- and every world
     coordinate in the mission is then a map coordinate too.

  2. Paint in the two table tops and the six chairs. The lidar scans at
     0.26 m and sees only legs -- 6 cm chair legs that a 1 degree beam misses
     as often as it hits -- so the costmaps flicker on the dining set, the
     0.57 m base would fit between a table's legs, and a planner would send
     the 1.45 m torso through the table. Each piece becomes its footprint.
"""

import sys

import cv2
import numpy as np
import yaml

SPAWN = (0.15, 0.20)
# World-frame rectangles the lidar cannot see: (cx, cy, size_x, size_y).
PAINT = [
    (-0.65, -1.43, 1.0, 1.8),     # dining table
    (-1.21, 1.31, 1.7, 0.6),      # side table under the north window
    # the six dining chairs, 0.5 x 0.5 seats over a 0.5 m leg square
    (-0.64, -0.50, 0.5, 0.5), (-0.64, -2.33, 0.5, 0.5),
    (-0.18, -1.87, 0.5, 0.5), (-1.09, -1.05, 0.5, 0.5),
    (-0.18, -1.05, 0.5, 0.5), (-1.12, -1.87, 0.5, 0.5),
]


def main():
    src, dst = sys.argv[1], sys.argv[2]
    meta = yaml.safe_load(open(src))
    img = cv2.imread(src.rsplit("/", 1)[0] + "/" + meta["image"], 0)
    h, w = img.shape
    res = meta["resolution"]
    ox = meta["origin"][0] + SPAWN[0]
    oy = meta["origin"][1] + SPAWN[1]
    for cx, cy, sx, sy in PAINT:
        c0 = int((cx - sx / 2 - ox) / res)
        c1 = int((cx + sx / 2 - ox) / res)
        r0 = h - 1 - int((cy + sy / 2 - oy) / res)
        r1 = h - 1 - int((cy - sy / 2 - oy) / res)
        img[max(r0, 0):min(r1, h) + 1, max(c0, 0):min(c1, w) + 1] = 0
    cv2.imwrite(dst + ".pgm", img)
    meta["image"] = dst.rsplit("/", 1)[-1] + ".pgm"
    meta["origin"] = [round(ox, 3), round(oy, 3), 0.0]
    with open(dst + ".yaml", "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    print(f"wrote {dst}.pgm ({w}x{h}) and {dst}.yaml, origin {meta['origin']}")


if __name__ == "__main__":
    main()
