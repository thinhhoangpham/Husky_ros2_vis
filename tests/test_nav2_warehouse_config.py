"""Guard the three deviations in config/nav2_warehouse.yaml against silent revert.

Pure YAML parsing, no ROS. The stock a200 config is read too, so the tests fail
loudly if the assumption they are built on (what stock actually says) changes.
"""
import os

import pytest
import yaml

REPO = "/home/thinhpham/Documents/Husky_viz"
CONFIG = f"{REPO}/config/nav2_warehouse.yaml"
STOCK = "/opt/ros/jazzy/share/clearpath_nav2_demos/config/a200/nav2.yaml"


def _params(path, node):
    with open(path) as f:
        doc = yaml.safe_load(f)
    return doc[node][node]["ros__parameters"]


@pytest.fixture(scope="module")
def global_costmap():
    return _params(CONFIG, "global_costmap")


@pytest.fixture(scope="module")
def local_costmap():
    return _params(CONFIG, "local_costmap")


def test_global_costmap_is_not_rolling(global_costmap):
    """The fix: fixed in `map`, so goals beyond a 20 m window are plannable."""
    assert global_costmap["rolling_window"] is False


def test_global_costmap_covers_the_saved_map(global_costmap):
    """Pre-map default only, but it should still not be smaller than the map."""
    with open(f"{REPO}/maps/warehouse_slam_map.yaml") as f:
        map_yaml = yaml.safe_load(f)
    res = map_yaml["resolution"]
    from PIL import Image
    with Image.open(os.path.join(f"{REPO}/maps", map_yaml["image"])) as img:
        w_px, h_px = img.size
    assert global_costmap["width"] >= w_px * res
    assert global_costmap["height"] >= h_px * res


def test_global_obstacle_layer_ranges_are_raised(global_costmap):
    scan = global_costmap["obstacle_layer"]["scan"]
    assert scan["obstacle_max_range"] == 20.0
    assert scan["raytrace_max_range"] == 25.0
    # clear at least as far as you mark
    assert scan["raytrace_max_range"] >= scan["obstacle_max_range"]


def test_local_costmap_stays_rolling(local_costmap):
    """The local costmap must keep following the robot."""
    assert local_costmap["rolling_window"] is True


def test_local_costmap_is_untouched():
    """Deviations are global-costmap-only; the local costmap must equal stock."""
    assert _params(CONFIG, "local_costmap") == _params(STOCK, "local_costmap")


def test_stock_global_costmap_still_has_the_problem():
    """If stock ever fixes this upstream, the override can be dropped."""
    stock = _params(STOCK, "global_costmap")
    assert stock["rolling_window"] is True
    assert (stock["width"], stock["height"]) == (20, 20)
