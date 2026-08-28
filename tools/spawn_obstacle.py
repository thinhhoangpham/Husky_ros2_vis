#!/usr/bin/env python3
"""Drop an unmapped solid box into a RUNNING Gazebo Harmonic sim, or remove it.

Purpose: a navigation test needs an obstacle that exists in physics and in the
lidar but NOT in the prior map, so that reaching the goal proves the *local*
costmap avoided it (same idea as tools/check_local_avoidance.py, but placeable
anywhere instead of relying on an arbol4 tree).

Frame: X Y are Gazebo world coordinates, which are also map-frame
coordinates. Verified, not assumed: tools/generate_park_maps.py rasterizes
maps/park_map.pgm straight from worlds/park.sdf world-space triangles with
only a translation (ORIGIN = (-55.0, -31.55), no rotation, no scale beyond
the 0.05 m/cell resolution), and maps/park_map.yaml carries that same origin
with zero yaw. So map == world, identity. That is also why
check_local_avoidance.py can hardcode a tree pose read out of park.sdf and
compare it against map-frame TF.

Box size: 1.0 m square footprint, 1.0 m tall (defaults).
  * half-width 0.5 m > the robot's 0.34 m half-width used as the clearance
    yardstick in check_local_avoidance.py, so an avoidance manoeuvre is a real
    detour and not noise.
  * the 2D lidar rides on bracket_0 above the top plate, roughly 0.4 m above
    ground; a 1.0 m tall box spanning ground to 1.0 m straddles that scan
    plane with margin, and stays under the costmaps' max_obstacle_height of
    2.0 m (config/nav2_park.yaml).
  * 1.0 m across is ~20 cells at the 0.05 m map resolution and wider than the
    0.8 m inflation_radius, so it cannot be inflated away.

Ground height is QUERIED, not assumed: the terrain collision triangles of the
ground models are loaded from the world SDF and the highest one covering
(X, Y) gives the surface z. The box is placed with its centre at
ground + height/2, so it rests ON the terrain. --ground-z overrides.

Gotcha #26: services are scoped by world name, so --world drives every call.
Gotcha #4: create/remove can report success for a no-op, so the result flag is
ignored and existence is re-checked against the live scene.

Usage:
  python3 tools/spawn_obstacle.py X Y [--size S] [--height H] [--name NAME]
                                      [--world WORLD] [--ground-z Z]
  python3 tools/spawn_obstacle.py --remove [--name NAME] [--world WORLD]

Exit codes: 0 verified, 1 verification failed, 2 bad usage / query failed.
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from xml.etree import ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from tools.sdf_geometry import world_triangles  # noqa: E402

# The two models that make up park's walkable surface, per
# tools/generate_park_maps.py GROUND_MODELS.
GROUND_MODELS = {"parque", "camino_parque"}

DEFAULT_NAME = "test_obstacle"
DEFAULT_SIZE = 1.0
DEFAULT_HEIGHT = 1.0

# gz lives inside the ROS tree and is not on the global PATH (CLAUDE.md,
# Environment), so every call is sourced first.
SOURCE = "source /opt/ros/jazzy/setup.bash"


def sh(cmd: str, timeout: float = 60.0) -> str:
    """Run one gz command under a sourced ROS environment, return stdout."""
    try:
        p = subprocess.run(["bash", "-lc", f"{SOURCE} && {cmd}"],
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return ""
    return p.stdout


def world_sdf_path(world: str) -> str:
    return os.path.join(REPO, "worlds", f"{world}.sdf")


def ground_height(world: str, x: float, y: float) -> float | None:
    """Highest ground-model collision triangle covering (x, y), in world z.

    Returns None when the world has no such model or the point is off the
    terrain - both of which mean the caller must not guess a z.
    """
    sdf = world_sdf_path(world)
    if not os.path.exists(sdf):
        print(f"FAIL  no world SDF at {sdf}; pass --ground-z explicitly")
        return None

    root = ET.parse(sdf).getroot()
    w = root.find("world")
    if w is None:
        print(f"FAIL  {sdf} has no <world>")
        return None
    all_names = {m.get("name", "") for m in w.findall("model")}
    ground = all_names & GROUND_MODELS
    if not ground:
        print(f"FAIL  {world} has none of the known ground models {sorted(GROUND_MODELS)}; "
              f"pass --ground-z explicitly")
        return None

    best = None
    for _name, tris in world_triangles(sdf, skip_models=all_names - ground):
        if tris.size == 0:
            continue
        lo = tris.min(axis=1)          # (k, 3) per-triangle min
        hi = tris.max(axis=1)          # (k, 3) per-triangle max
        cover = ((lo[:, 0] <= x) & (hi[:, 0] >= x) &
                 (lo[:, 1] <= y) & (hi[:, 1] >= y))
        if not cover.any():
            continue
        top = float(hi[cover, 2].max())
        best = top if best is None else max(best, top)

    if best is None:
        print(f"FAIL  ({x:.2f}, {y:.2f}) is not over {sorted(ground)} terrain - "
              f"nothing to stand on (gotcha #25)")
    return best


def box_sdf(name: str, size: float, height: float, x: float, y: float, z: float) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{size} {size} {height}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {height}</size></box></geometry>
        <material>
          <ambient>0.9 0.2 0.05 1</ambient>
          <diffuse>0.9 0.2 0.05 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def scene_models(world: str) -> list[str] | None:
    """Model names as the live renderer holds them (gotcha #20's technique).

    None means the service did not answer, i.e. no sim on this world.
    """
    out = sh(f"gz service -s /world/{world}/scene/info "
             f"--reqtype gz.msgs.Empty --reptype gz.msgs.Scene "
             f"--timeout 30000 --req ''")
    if not out.strip():
        return None
    names: list[str] = []
    depth_of_model = None
    depth = 0
    # The reply is protobuf text: track brace depth so only `name:` fields
    # directly inside a `model {` block are collected.
    for raw in out.splitlines():
        line = raw.strip()
        if line.startswith("model {"):
            if depth_of_model is None:
                depth_of_model = depth
            depth += 1
            continue
        if line.endswith("{"):
            depth += 1
            continue
        if line == "}":
            depth -= 1
            if depth_of_model is not None and depth == depth_of_model:
                depth_of_model = None
            continue
        if depth_of_model is not None and depth == depth_of_model + 1 \
                and line.startswith("name:"):
            names.append(line.split(":", 1)[1].strip().strip('"'))
    return names


def verify_present(world: str, name: str, want: bool) -> bool:
    models = scene_models(world)
    if models is None:
        print(f"FAIL  /world/{world}/scene/info did not answer - is the sim running "
              f"on world '{world}'? (gotcha #26)")
        return False
    present = name in models
    if present is want:
        print(f"PASS  '{name}' {'present in' if want else 'absent from'} "
              f"/world/{world} scene ({len(models)} models)")
        return True
    print(f"FAIL  '{name}' {'NOT present in' if want else 'STILL present in'} "
          f"/world/{world} scene despite a successful service reply (gotcha #4)")
    return False


def model_pose(name: str) -> str:
    out = sh(f"gz model -m {shlex.quote(name)} -p", timeout=30.0).strip()
    return out or "(pose query returned nothing)"


def do_spawn(args: argparse.Namespace) -> int:
    if args.size <= 0 or args.height <= 0:
        print("FAIL  --size and --height must be positive")
        return 2

    if verify_present(args.world, args.name, want=False) is False:
        # Either the scene service is dead or the name is already taken; both
        # block a clean spawn, and create would otherwise fail obscurely.
        print(f"      if that name is already taken, remove it first: python3 tools/spawn_obstacle.py --remove "
              f"--name {args.name} --world {args.world}")
        return 1

    if args.ground_z is not None:
        gz_ground = args.ground_z
        print(f"  ground z {gz_ground:.3f} (from --ground-z)")
    else:
        g = ground_height(args.world, args.x, args.y)
        if g is None:
            return 2
        gz_ground = g
        print(f"  ground z {gz_ground:.3f} (queried from worlds/{args.world}.sdf "
              f"terrain collision at x={args.x:.2f} y={args.y:.2f})")

    z = gz_ground + args.height / 2.0
    print(f"  box {args.size} x {args.size} x {args.height} m, centre z {z:.3f} "
          f"-> base rests on the terrain")

    sdf = box_sdf(args.name, args.size, args.height, args.x, args.y, z)
    req = "sdf: " + _pb_string(sdf)
    out = sh(f"gz service -s /world/{args.world}/create "
             f"--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean "
             f"--timeout 5000 --req {shlex.quote(req)}")
    print(f"  /world/{args.world}/create replied: {out.strip() or '(no reply)'} "
          f"- not trusted on its own (gotcha #4)")

    if not verify_present(args.world, args.name, want=True):
        return 1
    print(f"  verified pose: {model_pose(args.name)}")
    return 0


def do_remove(args: argparse.Namespace) -> int:
    req = f'name: "{args.name}" type: MODEL'
    out = sh(f"gz service -s /world/{args.world}/remove "
             f"--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean "
             f"--timeout 5000 --req {shlex.quote(req)}")
    print(f"  /world/{args.world}/remove replied: {out.strip() or '(no reply)'} "
          f"- not trusted on its own (gotcha #4)")
    return 0 if verify_present(args.world, args.name, want=False) else 1


def _pb_string(s: str) -> str:
    """Quote a python string as a protobuf text-format string literal."""
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("x", nargs="?", type=float, help="world/map-frame x (m)")
    ap.add_argument("y", nargs="?", type=float, help="world/map-frame y (m)")
    ap.add_argument("--size", type=float, default=DEFAULT_SIZE,
                    help=f"footprint edge length in m (default {DEFAULT_SIZE})")
    ap.add_argument("--height", type=float, default=DEFAULT_HEIGHT,
                    help=f"box height in m (default {DEFAULT_HEIGHT})")
    ap.add_argument("--name", default=DEFAULT_NAME,
                    help=f"model name (default {DEFAULT_NAME})")
    ap.add_argument("--world", default="park", help="Gazebo world name (default park)")
    ap.add_argument("--ground-z", type=float, default=None,
                    help="skip the terrain query and use this ground height (m)")
    ap.add_argument("--remove", action="store_true",
                    help="delete a previously spawned box by --name")
    args = ap.parse_args()

    if args.remove:
        if args.x is not None:
            print("FAIL  --remove takes no X Y")
            return 2
        print(f"==> removing '{args.name}' from world '{args.world}'")
        return do_remove(args)

    if args.x is None or args.y is None:
        print("FAIL  X and Y are required unless --remove is given")
        return 2
    print(f"==> spawning '{args.name}' into world '{args.world}' "
          f"at map/world ({args.x:.2f}, {args.y:.2f})")
    return do_spawn(args)


if __name__ == "__main__":
    sys.exit(main())
