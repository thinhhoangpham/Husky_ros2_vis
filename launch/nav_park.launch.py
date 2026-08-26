"""Autonomous navigation in park: GPS localization -> map servers -> nav2.

Stage order matters. Nav2 transitions planner_server to active and immediately
looks up map -> odom; if navsat_transform has no fix yet, nav2 comes up healthy
but useless - no crash, no error, goals silently ignored. park's GPS is 1 Hz,
so the window is wide. Run tools/check_nav2_ready.py before sending goals.

Controller ruling D2: clearpath_nav2_demos/launch/nav2.launch.py declares only
use_sim_time, setup_path and scan_topic, and hardcodes its params to
clearpath_nav2_demos/config/a200/nav2.yaml - config/nav2_park.yaml cannot be
injected through it. This file reproduces that wrapper's GroupAction
(namespace push + odom/tf remaps) directly around nav2_bringup's
navigation_launch.py instead, so our params file is used.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

REPO = "/home/thinhpham/Documents/Husky_viz"
NAMESPACE = "a200_0000"
FILTER_CONFIG = os.path.join(REPO, "config", "costmap_filter_info.yaml")
NAV2_CONFIG = os.path.join(REPO, "config", "nav2_park.yaml")
NAV2_BRINGUP_LAUNCH = "/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py"

LIFECYCLE_MAPS = ["map_server", "filter_mask_server", "costmap_filter_info_server"]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = [FILTER_CONFIG, {"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Stage 1 - global localization (map -> odom)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(REPO, "launch", "gps_localization.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        # Stage 2 - map, keepout mask, filter info
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             namespace=NAMESPACE, output="screen", parameters=params),
        Node(package="nav2_map_server", executable="map_server", name="filter_mask_server",
             namespace=NAMESPACE, output="screen", parameters=params),
        Node(package="nav2_map_server", executable="costmap_filter_info_server",
             name="costmap_filter_info_server", namespace=NAMESPACE,
             output="screen", parameters=params),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_maps", namespace=NAMESPACE, output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "autostart": True,
                          "node_names": LIFECYCLE_MAPS}]),

        # Stage 3 - nav2 proper, reproducing Clearpath's wrapper (ruling D2)
        # around nav2_bringup with our params file instead of Clearpath's.
        GroupAction([
            PushRosNamespace(NAMESPACE),
            SetRemap("/" + NAMESPACE + "/odom", "/" + NAMESPACE + "/platform/odom"),
            SetRemap("/tf", "/" + NAMESPACE + "/tf"),
            SetRemap("/tf_static", "/" + NAMESPACE + "/tf_static"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(NAV2_BRINGUP_LAUNCH),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("params_file", NAV2_CONFIG),
                    ("use_composition", "False"),
                    ("namespace", NAMESPACE),
                ],
            ),
        ]),
    ])
