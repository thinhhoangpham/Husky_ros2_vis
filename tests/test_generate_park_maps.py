import numpy as np
import pytest
import yaml
from PIL import Image

from tools.generate_park_maps import (
    GROUND_Z, ORIGIN, RES, SIZE, TERRAIN,
    rasterize_keepout, rasterize_obstacles, world_to_pixel, write_map,
)

REPO = "/home/thinhpham/Documents/Husky_viz"
SDF = f"{REPO}/worlds/park.sdf"


def test_world_to_pixel_origin_is_bottom_left():
    """origin maps to the BOTTOM-left pixel, i.e. the last image row."""
    col, row = world_to_pixel(ORIGIN[0], ORIGIN[1])
    assert col == 0
    assert row == SIZE[1] - 1


def test_world_to_pixel_is_metric():
    c0, r0 = world_to_pixel(0.0, 0.0)
    c1, r1 = world_to_pixel(1.0, 0.0)
    assert c1 - c0 == int(round(1.0 / RES))
    c2, r2 = world_to_pixel(0.0, 1.0)
    assert r0 - r2 == int(round(1.0 / RES))


def test_keepout_marks_outside_terrain_lethal():
    img = rasterize_keepout()
    a = np.asarray(img)
    assert a.shape == (SIZE[1], SIZE[0])
    # centre of the park is free (white)
    c, r = world_to_pixel(0.0, 0.0)
    assert a[r, c] == 255
    # 2 m beyond the north edge is lethal (black)
    c, r = world_to_pixel(TERRAIN[1] + 2.0, 0.0)
    assert a[r, c] == 0
    # 2 m beyond the west edge is lethal
    c, r = world_to_pixel(0.0, TERRAIN[3] + 2.0)
    assert a[r, c] == 0
    # just inside the north edge is free
    c, r = world_to_pixel(TERRAIN[1] - 0.5, 0.0)
    assert a[r, c] == 255


def test_obstacles_mark_a_known_tree_and_leave_the_spawn_clear():
    img = rasterize_obstacles(SDF, GROUND_Z + 0.10, GROUND_Z + 1.20)
    a = np.asarray(img)
    assert a.shape == (SIZE[1], SIZE[0])
    # something must be occupied
    assert (a == 0).sum() > 1000, "no obstacles rasterized at all"
    # the spawn pose must be free, or the robot starts inside an obstacle
    c, r = world_to_pixel(45.64, 0.02)
    win = a[r - 8:r + 9, c - 8:c + 9]
    assert (win == 255).all(), "spawn is not clear in the prior map"


def test_small_trees_are_absent_from_the_prior_map():
    """arbol4 is deliberately excluded so the LOCAL costmap must avoid it."""
    img = rasterize_obstacles(SDF, GROUND_Z + 0.10, GROUND_Z + 1.20)
    a = np.asarray(img)
    for x, y in [(5.67, 4.58), (-20.98, 2.64), (44.51, -7.19)]:
        c, r = world_to_pixel(x, y)
        win = a[r - 20:r + 21, c - 20:c + 21]     # +/- 1.0 m
        assert (win == 255).all(), f"arbol4 at ({x}, {y}) leaked into the prior map"


def test_large_trees_are_present_in_the_prior_map():
    """tree_8 is a 12.9 m specimen and stays in the map."""
    import xml.etree.ElementTree as ET
    world = ET.parse(SDF).getroot().find("world")
    pts = []
    for m in world.findall("model"):
        if any("tree_8" in (u.text or "") for u in m.iter("uri")):
            pose = m.find("pose")
            if pose is not None and pose.text:
                v = [float(t) for t in pose.text.split()]
                pts.append((v[0], v[1]))
    assert len(pts) >= 20
    img = rasterize_obstacles(SDF, GROUND_Z + 0.10, GROUND_Z + 1.20)
    a = np.asarray(img)
    marked = 0
    for x, y in pts:
        c, r = world_to_pixel(x, y)
        if (a[r - 8:r + 9, c - 8:c + 9] == 0).any():
            marked += 1
    assert marked >= len(pts) * 0.8, f"only {marked}/{len(pts)} tree_8 trunks mapped"


def test_obstacles_do_not_mark_the_ground():
    """The terrain and path are excluded by the height band, not by name;
    if the band is wrong the whole map fills in."""
    img = rasterize_obstacles(SDF, GROUND_Z + 0.10, GROUND_Z + 1.20)
    a = np.asarray(img)
    occupied_fraction = (a == 0).sum() / a.size
    assert occupied_fraction < 0.15, f"far too much occupied: {occupied_fraction:.3f}"


def test_write_map_emits_aligned_yaml(tmp_path):
    img = rasterize_keepout()
    write_map(img, "unit_test_map", str(tmp_path))
    meta = yaml.safe_load((tmp_path / "unit_test_map.yaml").read_text())
    assert meta["resolution"] == RES
    assert meta["origin"] == [ORIGIN[0], ORIGIN[1], 0.0]
    assert meta["negate"] == 0
    assert meta["image"] == "unit_test_map.pgm"
    reloaded = Image.open(tmp_path / "unit_test_map.pgm")
    assert reloaded.size == SIZE
