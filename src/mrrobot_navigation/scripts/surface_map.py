#!/usr/bin/env python3
"""Thin a map to the surfaces the lidar can see, for AMCL.

    ros2 run mrrobot_navigation surface_map.py maps/kitchen.yaml maps/kitchen_surface

The SLAM map's walls are 2-3 cells thick and its painted furniture is solid.
AMCL's likelihood field scores a beam by its distance to the NEAREST
occupied cell, so a beam ending anywhere inside a 15 cm wall scores the
same as one ending on its face: the estimate can slide towards a wall by
its thickness and lose nothing. Measured at the counter stations: 8-14 cm
along the lane after every hop, corrected by the scan matcher each time
and costing a re-approach.

Here only the occupied cells that border the free region the robot drives
in (a flood fill from `seed`) stay occupied -- what the lidar sees; the
rest of each wall and the inside of every painted piece become unknown,
which the likelihood field treats as empty. The costmaps keep the full map.
"""

import argparse
import os

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="map yaml")
    ap.add_argument("dst", help="output path without extension")
    ap.add_argument("--seed", nargs=2, type=float, default=(0.15, 0.20),
                    metavar=("X", "Y"), help="a point inside the driven region (map frame)")
    args = ap.parse_args()

    with open(args.src) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(os.path.join(os.path.dirname(args.src), meta["image"]), cv2.IMREAD_GRAYSCALE)
    if meta.get("negate", 0):
        img = 255 - img
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    occupied = img < 255 * (1 - meta["occupied_thresh"])
    free = img > 255 * (1 - meta["free_thresh"])

    # image rows run top-down, the map's y runs bottom-up
    h, w = img.shape
    cx, cy = int((args.seed[0] - ox) / res), h - 1 - int((args.seed[1] - oy) / res)
    region = free.astype(np.uint8)
    if not region[cy, cx]:
        raise SystemExit(f"seed {args.seed} is not in free space")
    cv2.floodFill(region, np.zeros((h + 2, w + 2), np.uint8), (cx, cy), 2)
    reachable = region == 2
    near = np.zeros_like(reachable)
    near[1:, :] |= reachable[:-1, :]
    near[:-1, :] |= reachable[1:, :]
    near[:, 1:] |= reachable[:, :-1]
    near[:, :-1] |= reachable[:, 1:]
    surface = occupied & near

    out = np.full_like(img, 205)          # unknown
    out[reachable] = 254
    out[surface] = 0
    cv2.imwrite(args.dst + ".pgm", out)
    meta["image"] = os.path.basename(args.dst) + ".pgm"
    meta["negate"] = 0
    with open(args.dst + ".yaml", "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print(f"{args.dst}.pgm: {int(occupied.sum())} occupied cells -> "
          f"{int(surface.sum())} surface cells, {int(reachable.sum())} free")


if __name__ == "__main__":
    main()
