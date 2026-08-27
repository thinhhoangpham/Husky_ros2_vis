"""Autonomous navigation in park: GPS localization -> map servers -> nav2.

Stage order matters. Nav2 transitions planner_server to active and immediately
looks up map -> odom; if navsat_transform has no fix yet, nav2 comes up healthy
but useless - no crash, no error, goals silently ignored. park's GPS is 1 Hz,
so the window is wide. Run tools/check_nav2_ready.py before sending goals.

Controller ruling D2: clearpath_nav2_demos/launch/nav2.launch.py declares only
use_sim_time, setup_path and scan_topic, and hardcodes its params to
clearpath_nav2_demos/config/a200/nav2.yaml - config/nav2_park.yaml cannot be
injected through it. This file reproduces that wrapper's GroupAction
(namespace push + odom/tf remaps) directly, so our params file is used.

Ruling D4 (2026-08-26): ONE lifecycle manager, not two.

    Previously this file ran its own `lifecycle_manager_maps` for the three map
    nodes alongside nav2_bringup's `lifecycle_manager_navigation`. Two managers
    bring their nodes up concurrently with no ordering between the groups, and
    the maps manager was observed stalling mid-sequence - map_server inactive,
    filter_mask_server inactive, costmap_filter_info_server unconfigured -
    while planner_server and controller_server went active anyway. The planner
    then had no map and no keepout mask, and nothing reported an error.

    The fix needs a single manager whose `node_names` puts the map nodes first,
    because nav2's LifecycleManager configures/activates strictly in list order
    and bonds each node before moving to the next.

    nav2_bringup/launch/navigation_launch.py cannot be made to do that:
      - its `lifecycle_nodes` list is hardcoded in the launch file, and
      - it passes `parameters=[{'autostart': ...}, {'node_names': ...}]` to the
        manager as explicit dicts, never a params file. An explicit launch
        parameter wins over a params-file value, and here there is no params
        file for that node at all - so a `lifecycle_manager_navigation:` block
        in config/nav2_park.yaml would simply never be read.
      - it exposes no condition/argument for skipping its own manager.

    So the include is gone and navigation_launch.py's non-composition
    `load_nodes` group is reproduced below verbatim (Jazzy nav2_bringup, from
    /opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py), minus its
    lifecycle manager. use_composition was already False here, so only that
    branch is needed.

    LIFECYCLE_NODES below must list exactly the nodes this file launches - no
    more, no fewer. A name that is never launched makes the manager block
    forever waiting for its change_state service; a launched node left off the
    list never gets configured. Keep it in sync with tools/check_nav2_ready.py.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

REPO = "/home/thinhpham/Documents/Husky_viz"
NAMESPACE = "a200_0000"
FILTER_CONFIG = os.path.join(REPO, "config", "costmap_filter_info.yaml")
NAV2_CONFIG = os.path.join(REPO, "config", "nav2_park.yaml")
RVIZ_CONFIG = os.path.join(REPO, "config", "nav_park.rviz")

# Strict startup order for the single lifecycle manager. The three map nodes
# come first so the map and the keepout mask are published and latched before
# planner_server/controller_server build their costmaps. The remaining ten are
# navigation_launch.py's `lifecycle_nodes`, in its order.
LIFECYCLE_NODES = [
    "map_server",
    "filter_mask_server",
    "costmap_filter_info_server",
    "controller_server",
    "smoother_server",
    "planner_server",
    "route_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
    "waypoint_follower",
    "docking_server",
]

# navigation_launch.py's remappings: map the fully qualified /tf names to
# relative ones so PushRosNamespace can prepend the namespace.
NAV2_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]
LOG_ARGS = ["--ros-args", "--log-level", "info"]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_params = [FILTER_CONFIG, {"use_sim_time": use_sim_time}]

    # Identical treatment to navigation_launch.py: our un-namespaced params
    # file is rewritten under the namespace as its root key.
    nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=NAV2_CONFIG,
            root_key=NAMESPACE,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    def nav2_node(package, executable, name=None, extra_remaps=()):
        kwargs = {}
        if name is not None:
            kwargs["name"] = name
        return Node(
            package=package,
            executable=executable,
            output="screen",
            respawn=False,
            respawn_delay=2.0,
            parameters=[nav2_params],
            arguments=LOG_ARGS,
            remappings=NAV2_REMAPS + list(extra_remaps),
            **kwargs,
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        # Stage 1 - global localization (map -> odom)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(REPO, "launch", "gps_localization.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        # Stage 2 - map, keepout mask, filter info. Launched here but managed
        # by the single lifecycle manager in Stage 3 (ruling D4).
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             namespace=NAMESPACE, output="screen", parameters=map_params),
        Node(package="nav2_map_server", executable="map_server", name="filter_mask_server",
             namespace=NAMESPACE, output="screen", parameters=map_params),
        Node(package="nav2_map_server", executable="costmap_filter_info_server",
             name="costmap_filter_info_server", namespace=NAMESPACE,
             output="screen", parameters=map_params),

        # Stage 3 - nav2 proper, reproducing Clearpath's wrapper (ruling D2)
        # and nav2_bringup's node set (ruling D4) with our params file.
        GroupAction([
            PushRosNamespace(NAMESPACE),
            SetParameter("use_sim_time", use_sim_time),
            SetRemap("/" + NAMESPACE + "/odom", "/" + NAMESPACE + "/platform/odom"),
            SetRemap("/tf", "/" + NAMESPACE + "/tf"),
            SetRemap("/tf_static", "/" + NAMESPACE + "/tf_static"),
            # No cmd_vel remap: velocity_smoother publishes cmd_vel_smoothed and
            # collision_monitor forwards it to cmd_vel (stock Clearpath wiring).
            # A remap bypassing the monitor used to live here, needed only while
            # the 2D lidar's 360 deg sweep saw the robot's own sensor arch; the
            # sweep is now +-135 deg. See config/nav2_park.yaml.

            nav2_node("nav2_controller", "controller_server",
                      extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
            nav2_node("nav2_smoother", "smoother_server", "smoother_server"),
            nav2_node("nav2_planner", "planner_server", "planner_server"),
            nav2_node("nav2_route", "route_server", "route_server"),
            nav2_node("nav2_behaviors", "behavior_server", "behavior_server",
                      extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
            nav2_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
            nav2_node("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
            nav2_node("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother",
                      extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
            nav2_node("nav2_collision_monitor", "collision_monitor", "collision_monitor"),
            nav2_node("opennav_docking", "opennav_docking", "docking_server"),
        ]),

        # Optional view. Its own group: it needs the namespace and the tf
        # remaps, but none of Stage 3's topic remaps. The .rviz file names
        # every display topic absolutely, so PushRosNamespace only affects
        # rviz2's own /tf and /tf_static subscriptions - which is the point,
        # since this stack publishes tf under the namespace, not globally.
        GroupAction([
            PushRosNamespace(NAMESPACE),
            Node(package="rviz2", executable="rviz2", name="rviz2",
                 output="screen", arguments=["-d", RVIZ_CONFIG],
                 parameters=[{"use_sim_time": use_sim_time}],
                 remappings=NAV2_REMAPS,
                 condition=IfCondition(LaunchConfiguration("rviz"))),
        ]),

        # The one manager. Outside the Stage 3 group on purpose: it needs the
        # namespace but none of that group's topic remaps.
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", namespace=NAMESPACE,
             output="screen", arguments=LOG_ARGS,
             parameters=[{"use_sim_time": use_sim_time,
                          "autostart": autostart,
                          "node_names": LIFECYCLE_NODES}]),
    ])
