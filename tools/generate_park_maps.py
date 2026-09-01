#!/usr/bin/env python3
"""Generate the nav2 prior obstacle map for park.

Origin, resolution and size are defined here as the single source of truth.

Nav2 image convention with negate=0: black (0) is occupied, white (255) free.

Usage:
    python3 tools/generate_park_maps.py [--out maps]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import yaml
from PIL import Image, ImageDraw

from tools.sdf_geometry import models_using_mesh_dir, world_triangles

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- Single source of truth for raster geometry. Keep in sync with the spec.
RES = 0.05                              # metres per cell
ORIGIN = (-55.0, -31.55)                # world coords of the BOTTOM-LEFT cell
SIZE = (2200, 1200)                     # (width, height) in cells
TERRAIN = (-50.0, 50.0, -26.55, 23.45)  # xmin, xmax, ymin, ymax
GROUND_Z = 2.98891

# Ground surfaces: excluded by name as well as by the height band, because
# rasterizing 131k terrain triangles only to discard them wastes ~30 s.
GROUND_MODELS = {"parque", "camino_parque"}

# Small canopy trees (3.10 m across, 4.11 m tall) are deliberately kept OUT of
# the prior map so the global planner routes through them and the local costmap
# has to avoid them from live lidar. This is the obstacle-avoidance demo, and
# it is an intentional divergence between the prior map and reality.
# tree_8 (12.9 m specimens), tables, bins, lamps and poles all stay in.
PRIOR_EXCLUDE_MESH_DIRS = {"arbol4"}
# linea1 (power line) is excluded because the ported world places it far too low:
# its geometry spans 0.55 m BELOW ground to only 3.05 m above, so the cables hang
# at robot height and rasterise as 60 m walls across the park. In reality a power
# line is overhead infrastructure the robot drives under. Faithful to the source
# world, wrong as a ground obstacle.

FREE, OCCUPIED = 255, 0


def world_to_pixel(x: float, y: float) -> tuple[int, int]:
    """World metres -> (col, row). Row 0 is the TOP of the image."""
    # floor, not round: nav2's map_server treats cell n as covering
    # [origin + n*res, origin + (n+1)*res). The epsilon absorbs float noise
    # like (1.0 - -31.55) / 0.05 == 650.9999999999999 without shifting the
    # convention to round-to-nearest (which would introduce a half-cell offset).
    col = int(math.floor((x - ORIGIN[0]) / RES + 1e-9))
    row = SIZE[1] - 1 - int(math.floor((y - ORIGIN[1]) / RES + 1e-9))
    return col, row


def prior_skip_models(sdf_path: str) -> set[str]:
    """Models left out of the prior map: ground surfaces plus small trees."""
    skip = set(GROUND_MODELS)
    for d in PRIOR_EXCLUDE_MESH_DIRS:
        skip |= models_using_mesh_dir(sdf_path, d)
    return skip


def rasterize_obstacles(sdf_path: str, z_lo: float, z_hi: float) -> Image.Image:
    """Black where collision geometry intersects the [z_lo, z_hi] band."""
    img = Image.new("L", SIZE, FREE)
    draw = ImageDraw.Draw(img)
    skip = prior_skip_models(sdf_path)
    print(f"    excluded from prior map: {len(skip) - len(GROUND_MODELS)} small trees "
          f"+ {len(GROUND_MODELS)} ground surfaces")
    total = kept = 0
    for name, T in world_triangles(sdf_path, skip_models=skip):
        if len(T) == 0:
            continue
        total += len(T)
        zmin = T[:, :, 2].min(axis=1)
        zmax = T[:, :, 2].max(axis=1)
        band = T[(zmax >= z_lo) & (zmin <= z_hi)]
        kept += len(band)
        for tri in band:
            pts = [world_to_pixel(float(v[0]), float(v[1])) for v in tri]
            draw.polygon(pts, fill=OCCUPIED)
    print(f"    triangles: {total:,} total, {kept:,} in band [{z_lo:.2f}, {z_hi:.2f}]")
    return img


def write_map(img: Image.Image, stem: str, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    pgm = os.path.join(out_dir, f"{stem}.pgm")
    img.save(pgm)
    meta = {
        "image": f"{stem}.pgm",
        "mode": "trinary",
        "resolution": RES,
        "origin": [ORIGIN[0], ORIGIN[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(os.path.join(out_dir, f"{stem}.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh, default_flow_style=False, sort_keys=False)
    occ = (np.asarray(img) == OCCUPIED).sum()
    print(f"    wrote {stem}.pgm / {stem}.yaml  ({occ:,} occupied cells)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sdf", default=os.path.join(REPO, "worlds", "park.sdf"))
    ap.add_argument("--out", default=os.path.join(REPO, "maps"))
    args = ap.parse_args()

    print(f"==> obstacle map from {args.sdf}")
    obstacles = rasterize_obstacles(args.sdf, GROUND_Z + 0.10, GROUND_Z + 1.20)
    write_map(obstacles, "park_map", args.out)

    print(f"==> raster: {SIZE[0]}x{SIZE[1]} @ {RES} m, origin {ORIGIN}")


if __name__ == "__main__":
    main()
