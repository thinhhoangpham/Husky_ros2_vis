"""park, end to end, on the stock Clearpath launchers: world + robot + nav2 + GPS.

Stock code does the world and the robot; this file wires the navigation stack
onto it directly. Nothing here includes another project launch file - the
Stages below are written out so this file is a single, readable account of what
runs, rather than a wrapper whose behaviour lives somewhere else.

What is stock, and included verbatim
------------------------------------
  clearpath_gz/launch/gz_sim.launch.py       world + clock bridge
  clearpath_gz/launch/robot_spawn.launch.py  generators, platform/sensor
                                             services, robot spawn, control
  clearpath_viz/launch/view_navigation.launch.py   the nav2 rviz view

  `simulation.launch.py` is the wrapper normally used for the first two, and it
  is the *only* thing in that chain that rejects `world:=park` - it declares
  `world` with a six-name `choices` whitelist (gotcha #6). The two sub-launchers
  it wraps declare `world` with no `choices` at all, so they take park directly.
  Skipping the wrapper is therefore not a fork of stock code; it is using stock
  code one level down.

What is project code, and why upstream cannot supply it
-------------------------------------------------------
  1. Mesh resolution for a non-stock world. gz_sim.launch.py:110 rebuilds
     GZ_SIM_RESOURCE_PATH from `os.path.join(p, 'share')` for every
     AMENT_PREFIX_PATH entry and *discards* any inherited
     GZ_SIM_RESOURCE_PATH - so exporting that variable is useless and the only
     hook is AMENT_PREFIX_PATH itself. REPO/gz is shaped like an ament prefix
     (`gz/share/<model>/`) precisely so it can be prepended there. Getting this
     wrong is not a missing texture, it is gotcha #17: Harmonic aborts the whole
     world load on a single unresolvable mesh URI.

     Ordering ruling: AppendEnvironmentVariable is safe *here* and must stay
     ahead of the gz_sim include in this list. LaunchContext.environment is
     literally `os.environ` (launch/launch_context.py:205), so the action
     mutates the same dict the stock file reads; and the stock file's
     `os.getenv` runs inside its generate_launch_description(), which
     IncludeLaunchDescription.execute() calls at *visit* time, not at
     construction time. Actions in a LaunchDescription are visited in list
     order, so the write strictly precedes the read. `ros2 launch` does load
     the included file once more, early and without context, to harvest its
     launch arguments (LaunchDescription.get_launch_arguments_with_... ->
     LaunchDescriptionSource.try_get_launch_description_without_context), but
     that call deliberately does not cache its result
     (launch_description_source.py:63-75), so the environment-blind copy is
     thrown away and the real one is rebuilt after the append.

  2. Stage 1, global localization (map -> odom). There is none in stock
     Clearpath - `navsat_transform` appears in no clearpath_* package; stock
     offers AMCL (`localization.launch.py`) or slam_toolbox (`slam.launch.py`)
     only. robot_localization's own dual_ekf_navsat_example.launch.py is
     unusable as an include: it hardcodes its params directory (:26) and starts
     a second `ekf_filter_node_odom` that would contend with Clearpath's
     generated local EKF for odom -> base_link. Only ONE ekf_node runs here,
     `ekf_node_map`; the odom -> base_link edge stays with the generated one.

     `imu_enu` gets an explicit ros_gz_bridge rather than a robot.yaml sensor
     entry: the generator gates bridging on a declared sensor (gotcha #7), but
     declaring it as a stock `imu_1` would also make the generator fuse it into
     the *local* EKF, double-counting yaw rate on odom -> base_link.

     The radio topics get an explicit ros_gz_bridge too, for the same reason
     and only under `localization:=rssi`. tools/rssi_localization_node.py
     pings the towers on /broker/msgs (ROS -> gz) and reads the robot's
     replies on /husky/rx (gz -> ROS); nothing else in this chain bridges
     them, so without it the node subscribes to a topic with no publisher and
     silently never solves (gotcha #34's family). Only those two topics are
     bridged: the node reads no other rx topic, and the compass is not part of
     this backend - config/rssi_localization.yaml fuses only `rssi/pose` and
     `imu_enu`. Both names are GLOBAL, not `a200_0000`-scoped; the gz side
     fixes them. The `gps` backend needs none of this, so the bridge is not
     started for it.

     tools/rssi_viz.py starts alongside them, under `localization:=rssi`
     only, and is the visualisation of that same measurement: it pings the
     towers itself and publishes a visualization_msgs/MarkerArray on
     /a200_0000/rssi_viz (towers, live links, the trilaterated fix against
     ground truth). It is started as a bare Node with NO namespace and NO
     TF remaps on purpose - the script injects its own
     /tf:=/a200_0000/tf remaps when the caller supplies neither a TF remap
     nor `__ns:=`, and passing either from here would suppress that and
     leave its TransformListener bound to a global /tf nothing publishes.
     Only `use_sim_time` is passed; --rate keeps the script's own 1.0 Hz
     default. Its markers are shown by ADDING one MarkerArray display to
     the stock nav2 view, not by running a different viewer: the stock
     clearpath_viz view_navigation.launch.py is always the thing that starts
     rviz, and under `rssi` it is handed config/nav2_rssi.rviz - a verbatim
     copy of stock nav2.rviz plus that one display, so all seven stock Tools
     (SetInitialPose, PublishPoint, nav2_rviz_plugins/GoalTool included)
     remain. Standing rule for this file: the stock launchers are ADDED TO,
     never replaced by a project equivalent. Exactly one rviz starts either
     way, and `rviz:=false` starts none.

     PREREQUISITE: a robot config that declares the radio.
     robot_configs/robot_default.yaml - the config the stock chain documents
     applying - has none, so the bridge would carry nothing. Use
     robot_configs/robot_radio.yaml (extras_radio.urdf.xacro ->
     comms.urdf.xacro), and a world carrying the RFComms system (park.sdf:17).

  3. Stage 3, nav2 with our params (ruling D2).
     clearpath_nav2_demos/launch/nav2.launch.py declares only use_sim_time,
     setup_path and scan_topic and hardcodes its params to
     clearpath_nav2_demos/config/a200/nav2.yaml, so config/nav2_park.yaml
     cannot be injected through it and it must not be included. Stage 3 below
     reproduces that wrapper's GroupAction (namespace push + odom/tf remaps)
     around nav2_bringup/launch/navigation_launch.py's non-composition node
     set, with our params file.

  Ruling D3 runs through all of it: the Clearpath stack remaps /tf and
  /tf_static to /a200_0000/tf and /a200_0000/tf_static throughout
  (clearpath_control/control.launch.py, .../localization.launch.py, the
  generated platform-service.launch.py). Every node started here carries the
  same remap, or its transforms are published where nothing in this stack
  reads them - a silent failure in gotcha #34's family.

  Ruling D4: ONE lifecycle manager, map nodes first. Two managers bring their
  groups up concurrently with no ordering between them; map_server was observed
  left inactive while planner_server went active anyway, giving a planner with
  no map and no error. nav2_bringup's navigation_launch.py cannot be made to
  cooperate - its `lifecycle_nodes` list is hardcoded, it passes autostart and
  node_names to the manager as explicit dicts (which beat any params file), and
  it exposes no way to skip its own manager - which is the other half of why it
  is reproduced rather than included. LIFECYCLE_NODES below must list exactly
  the nodes this file launches: a name that is never launched makes the manager
  block forever on its change_state service, and a launched node left off the
  list never gets configured. Keep it in sync with tools/check_nav2_ready.py.

  Do not add slam_toolbox anywhere in here. Stage 1 publishes map -> odom from
  `ekf_node_map`; a second producer on that edge does not error, it flickers
  (gotcha #34 family).

The readiness gate
------------------
  Nav2 autostarts, activates planner_server and immediately looks up
  map -> odom. Started at t=0 alongside Gazebo it comes up healthy and ignores
  every goal silently (gotcha #34) - and in a single composed launch there is
  no operator starting the world first, so t=0 means "while park's meshes are
  still loading". This file must not rely on that being noticed.

  The gate is event-driven, not timed. Stock robot_spawn.launch.py's last
  action is a `ros_gz_sim create` process; `create` calls the world's
  /world/park/create service and blocks until Gazebo answers, which cannot
  happen until the world has finished loading and the server is servicing
  requests. Its exit is therefore a real signal - "park is up and the robot
  exists" - and Stages 1-3 hang off it via RegisterEventHandler(OnProcessExit).

  That Node object is constructed inside the stock file's OpaqueFunction and is
  not reachable from here, so the handler uses OnProcessExit's *callable*
  target_action form (on_action_event_base.py:79-80) and matches the process by
  the basename of its command, guarded to fire once.

  Residual risk, stated rather than hidden: the event proves the world is
  stepping, not that a GPS fix has arrived. It does not have to - Stage 1's
  first fix arrives within ~0.5 s of its bridge coming up (2 Hz GPS) while the
  lifecycle manager takes seconds to walk eight nodes through configure and
  activate. tools/check_nav2_ready.py stays the confirmation before a goal.
  Failure is loud rather than silent in the other direction too: if `create`
  never exits, no nav2 node ever appears at all.

The GUI gate (why the robot spawns late)
----------------------------------------
  A second race sits in front of the one above, and it is the GUI's. The
  Gazebo GUI subscribes to the scene once at startup and then spends ~4 s
  loading park's meshes (~97 models, ~221 MB of textures). Stock
  robot_spawn.launch.py's `ros_gz_sim create` fires the moment the SERVER
  answers /world/park/create - which happens while the GUI is still loading -
  so the creation event is missed and the GUI's scene stays stale
  indefinitely. Everything else is healthy: physics has the robot,
  `gz model -m a200_0000/robot -p` returns its pose, platform/odom publishes,
  nav2 localizes. The window simply has no robot in it.

  It cannot be detected after the fact. `/world/park/scene/info` is served by
  SceneBroadcaster in the SERVER (gotcha #20), so it reports the server's
  scene graph, not what the GUI drew; and `gz service -l | grep ^/gui` offers
  only camera/view/copy/paste/screenshot/follow/move_to - nothing re-requests
  the scene. Restarting the GUI alone crashes the session, and stock
  gz_sim.launch.py builds `gz_args` itself with no hook for `-s`, so there is
  no server-first path either.

  So it is prevented, by gating the robot_spawn include on the GUI having
  loaded. The signal is `/gui/copy`: gz-gui advertises a plugin's services
  from that plugin's LoadConfig, and CopyPaste is the LAST plugin in
  clearpath_gz/config/gui.config to advertise any `/gui/*` service (:1003;
  Teleop follows at :1071 and advertises none). Its appearance therefore
  implies every earlier plugin has loaded, GzSceneManager (:157) included.
  `libCopyPaste.so` is the only binary exporting the name, checked with
  `strings` against the gz_sim_vendor plugin directory.

  Elapsed time was deliberately not used. A fixed delay is what commit
  0319914 tried (a `spawn_delay` TimerAction, 15 s for park) and it was
  reverted the same day (a82d906); 09599e5 tried the other end - a
  server-side renderer check in sim.py phase 4 with a retry loop - and was
  reverted too (c52b316), because scene/info is not GUI-authoritative. Do not
  re-propose either. The gate here polls a real condition under a hard
  deadline, which is the form CLAUDE.md sanctions.

  The deadline is GUI_READY_DEADLINE_S = 90.0, matching scripts/sim.py's
  LAUNCH_DEADLINE for the same event class (park's world coming up); park's
  own mesh load is ~4 s, so this is ~22x headroom and only a genuinely dead
  GUI reaches it. On timeout the gate exits nonzero and the handler emits
  Shutdown - the launch dies with a stated reason rather than proceeding to a
  spawn that would be missed again.

  Residual risk, stated rather than hidden: `/gui/copy` proves the GUI's
  plugins finished loading, not that the render thread has finished building
  the scene - the scene is created lazily on the render thread after plugin
  load, so a mesh-loading tail can still, in principle, extend past the gate.
  There is no GUI-side "scene ready" service to poll instead (the list above
  is exhaustive), so this is a narrowing of the window, not a proof. The
  operator's confirmation is still the window itself.

Caveats the operator owns
-------------------------
  * NO CONTROLLER-SPAWNER RECOVERY. Stock has none, and park loses the spawner
    race in 42% of runs (gotcha #27) - 11 of 43 recorded runs ended with no
    controllers active at all, presenting as a robot that will not move while
    every other check is green. scripts/sim.py Phase 3 handles this; this file
    does not. Check it by hand (the `ros2 control` CLI is not installed here,
    gotcha #33):
      ros2 service call /a200_0000/controller_manager/list_controllers \
        controller_manager_msgs/srv/ListControllers "{}"
    and re-run the spawner with --switch-timeout 30 if either controller is
    missing.

  * SPAWN POSE. Stock robot_spawn.launch.py has no WORLD_SPAWN_POSES table and
    defaults z to 0.15; park's ground is at z~2.99, so the stock default drops
    the robot through the terrain and it falls forever (gotcha #23). park's
    authored pose is the default of the x/y/z/yaw arguments below.

No scoping - and why re-adding it breaks the launch
--------------------------------------------------
  Every stock include is handed its arguments explicitly, and NONE of them is
  wrapped in a GroupAction at all - conditions ride on the includes and on the
  readiness event handler themselves. Do not wrap them in a GroupAction and do
  not add `scoped=True` anywhere in this file (GroupAction scopes by default,
  so a group added "just to carry a condition" re-arms the abort below).

  Scoping is unnecessary. An explicitly passed launch argument overrides an
  inherited configuration: IncludeLaunchDescription.execute returns
  `[*set_launch_configuration_actions, launch_description]`
  (include_launch_description.py:243), i.e. one SetLaunchConfiguration per
  passed argument, visited immediately before the included description - while
  DeclareLaunchArgument.execute applies its default only `if self.name not in
  context.launch_configurations` (declare_launch_argument.py:205). So the two
  leaks the scoping was defending against cannot happen: gz_sim's absolute
  `world` path does leak into this scope, but robot_spawn is handed the bare
  name `park` and that wins, keeping its `/world/park/model/...` topic
  prefixes (robot_spawn.launch.py:102,:108); and view_robot stays off because
  robot_spawn is handed `rviz:=false` explicitly, so there is no third rviz
  window.

  Scoping is also actively harmful. Stock robot_spawn.launch.py declares
  `generate` (:54) and reads it from `IfCondition(generate)` at
  :131,:140,:149,:158 and from `LaunchConfiguration('generate')` at :208,:212 -
  but its four generator nodes are chained through its own
  RegisterEventHandler(OnProcessExit(...)) (:162-186), so those conditions are
  evaluated long AFTER the include has been visited. A scope pops the moment
  the group's actions are visited (GroupAction.get_sub_entities appends
  PopLaunchConfigurations right after them), so by then `generate` is gone and
  launch aborts ~0.1 s in with "launch configuration 'generate' does not
  exist" - before platform, control or `create` ever start, so no robot spawns
  and the readiness gate below never fires. A plain `ros2 launch
  robot_spawn.launch.py` works only because the DeclareLaunchArgument lands in
  the ROOT scope, which nothing pops; not scoping puts it in OUR root scope,
  the same way. Passing `generate` through launch_arguments does NOT fix it -
  that is a SetLaunchConfiguration inside the very scope that pops. Verified by
  bisecting on a synthetic pair of launch files: a scope at EITHER level, the
  include's or an enclosing group's, is enough to abort.

  The one cost is confinement in the other direction: robot_spawn's `rviz`,
  pinned false here, leaks into our root scope. `nav_rviz` is captured from
  `rviz` before that group so the nav2 view is decided from a name nothing
  downstream writes.

Usage:
    ros2 launch launch/park_stock.launch.py
    ros2 launch launch/park_stock.launch.py localization:=rssi rviz:=false

    # navigation stages only, against a sim that is already running:
    ros2 launch launch/park_stock.launch.py world_and_robot:=false rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

REPO = "/home/thinhpham/Documents/Husky_viz"
NAMESPACE = "a200_0000"

# Shaped as an ament prefix (gz/share/<model>/) so it can be prepended to
# AMENT_PREFIX_PATH - the only channel stock gz_sim.launch.py leaves open.
GZ_PREFIX = os.path.join(REPO, "gz")

# Absolute, and WITHOUT the .sdf suffix: gz_sim.launch.py:90 builds gz_args as
# `<world> + '.sdf'`.
PARK_WORLD_PATH = os.path.join(REPO, "worlds", "park")

# Bare name, and NOT the same value as above: robot_spawn.launch.py:102,:108
# splice `world` into `/world/<world>/model/.../sensor/` topic prefixes, and
# park.sdf declares `<world name='park'>`.
PARK_WORLD_NAME = "park"

# park's authored spawn pose (gotcha #23).
PARK_SPAWN = {"x": "45.64", "y": "0.02", "z": "3.3", "yaw": "2.6132"}

GPS_CONFIG = os.path.join(REPO, "config", "gps_localization.yaml")
RSSI_CONFIG = os.path.join(REPO, "config", "rssi_localization.yaml")
RSSI_NODE = os.path.join(REPO, "tools", "rssi_localization_node.py")
RSSI_VIZ_NODE = os.path.join(REPO, "tools", "rssi_viz.py")
# Stock clearpath_viz nav2.rviz plus one MarkerArray display for
# /a200_0000/rssi_viz - added to, not replacing it, so every stock display
# and all seven stock Tools (SetInitialPose, PublishPoint and
# nav2_rviz_plugins/GoalTool among them) survive. Absolute on purpose:
# view_navigation.launch.py:66 builds PathJoinSubstitution([<pkg>/rviz,
# config]), and an absolute second component makes the join yield that path
# unchanged - so the stock launcher serves our config with no fork.
NAV2_RSSI_RVIZ = os.path.join(REPO, "config", "nav2_rssi.rviz")
MAP_CONFIG = os.path.join(REPO, "config", "map_server.yaml")
NAV2_CONFIG = os.path.join(REPO, "config", "nav2_park.yaml")

# Stage 1 backends. Each must provide the same contract: map -> odom
# published, by exactly one node. `choices` on the launch argument is what
# makes a typo fail loudly instead of quietly launching no Stage 1 at all -
# which presents as nav2 coming up healthy and ignoring every goal (gotcha
# #34). Both run an ekf_node_map with world_frame: map and publish_tf: true,
# so exactly one may ever be active; the conditions below are mutually
# exclusive by construction, being EqualsSubstitution on one configuration.
LOCALIZATION_BACKENDS = ["gps", "rssi"]

# Strict startup order for the single lifecycle manager (ruling D4).
# map_server first so the prior map is published and latched before
# planner_server/controller_server build their costmaps; the rest are
# navigation_launch.py's `lifecycle_nodes` in its order, minus route_server,
# smoother_server and docking_server (removed 2026-08-27 as unused) and minus
# filter_mask_server / costmap_filter_info_server (removed with the keepout
# filter - do not reintroduce them).
LIFECYCLE_NODES = [
    "map_server",
    "controller_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
    "waypoint_follower",
]

# Ruling D3, Stage 1 form: absolute source to absolute namespaced target,
# because these nodes are not inside a PushRosNamespace group.
TF_REMAPS = [
    ("/tf", "/" + NAMESPACE + "/tf"),
    ("/tf_static", "/" + NAMESPACE + "/tf_static"),
]

# Ruling D3, Stage 3 form: navigation_launch.py's own remappings, mapping the
# fully qualified names to relative ones so PushRosNamespace prepends the
# namespace.
NAV2_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]

LOG_ARGS = ["--ros-args", "--log-level", "info"]

# The GUI gate - see the docstring. `/gui/copy` is advertised by CopyPaste,
# the last plugin in clearpath_gz/config/gui.config to advertise any /gui/*
# service, so its presence means every GUI plugin has loaded.
GUI_READY_SERVICE = "/gui/copy"

# Matches scripts/sim.py's LAUNCH_DEADLINE for the same event class. park's
# mesh load is ~4 s, so only a dead GUI reaches this.
GUI_READY_DEADLINE_S = 90.0

GUI_READY_SCRIPT = f"""
end=$((SECONDS + {GUI_READY_DEADLINE_S:.0f}))
until gz service -l 2>/dev/null | grep -qx '{GUI_READY_SERVICE}'; do
  if [ "$SECONDS" -ge "$end" ]; then
    echo "gui gate: {GUI_READY_SERVICE} absent after {GUI_READY_DEADLINE_S:.0f} s" >&2
    exit 1
  fi
  sleep 0.5
done
echo "gui gate: {GUI_READY_SERVICE} present - GUI plugins loaded, spawning the robot"
"""


def _is_robot_spawn_process(action) -> bool:
    """Match stock robot_spawn.launch.py's `ros_gz_sim create` process.

    The Node object is built inside that file's OpaqueFunction and cannot be
    referenced from here, so the match is on the running command instead.
    """
    details = getattr(action, "process_details", None)
    if not details:
        return False
    cmd = details.get("cmd") or []
    return bool(cmd) and os.path.basename(cmd[0]) == "create"


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    world = LaunchConfiguration("world")
    world_and_robot = LaunchConfiguration("world_and_robot")
    localization = LaunchConfiguration("localization")

    pkg_clearpath_gz = get_package_share_directory("clearpath_gz")
    pkg_clearpath_viz = get_package_share_directory("clearpath_viz")

    def stock(package_share, name, arguments, condition=None):
        """Include a stock launch file with its arguments passed explicitly.

        Deliberately NOT wrapped in a GroupAction - see the docstring. A
        condition is carried by the include itself rather than by a group.
        """
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", name)),
            launch_arguments=arguments.items(),
            condition=condition,
        )

    def backend(name):
        """Condition selecting exactly one Stage 1 backend."""
        return IfCondition(EqualsSubstitution(localization, name))

    def imu_enu_bridge():
        """imu_enu's explicit bridge - see point 2 of the docstring (gotcha #7).

        Publishes no TF, so no TF_REMAPS. One instance per backend, and the
        backends are mutually exclusive, so there is only ever one.
        """
        return Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="imu_enu_gz_bridge",
            output="screen",
            arguments=[
                "/a200_0000/sensors/imu_enu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )

    def radio_bridge():
        """The RF comms bridge - rssi backend only, see the docstring.

        Global topic names, deliberately not under NAMESPACE. `]` is
        ROS -> gz (the outbound ping), `[` is gz -> ROS (the reply); a
        reversed marker gives a bridge that advertises and carries nothing.
        The condition rides on this Node directly - no GroupAction here.
        """
        return Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="radio_gz_bridge",
            output="screen",
            arguments=[
                "/broker/msgs@ros_gz_interfaces/msg/Dataframe]gz.msgs.Dataframe",
                "/husky/rx@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=backend("rssi"),
        )

    def rssi_viz():
        """The RSSI marker publisher - rssi backend only, see the docstring.

        Started exactly like rssi_localization_node.py (absolute `executable`;
        tools/ is not an ament package), but deliberately WITHOUT `namespace`
        and without TF_REMAPS: the script supplies its own
        /tf:=/a200_0000/tf remaps unless the caller passed a TF remap or a
        `__ns:=`, and launch_ros emits `-r __ns:=...` for `namespace`, which
        would suppress them. Its topics (/broker/msgs, /husky/rx,
        /a200_0000/rssi_viz) are all absolute already, so the namespace buys
        nothing. It publishes no TF. The condition rides on this Node
        directly - no GroupAction here.
        """
        return Node(
            executable=RSSI_VIZ_NODE,
            name="rssi_viz",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=backend("rssi"),
        )

    def ekf_node_map(config):
        """The single map -> odom publisher. Never a second one."""
        return Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node_map",
            namespace=NAMESPACE,
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
            remappings=[
                ("odometry/filtered", "platform/odom/filtered_map"),
            ] + TF_REMAPS,
        )

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

    def navigation_stages():
        """Stages 1-3, started once the world is up. See the readiness gate."""
        return [
            # Stage 1a - GPS. navsat_transform consumes the GLOBAL filter's
            # own estimate (platform/odom/filtered_map), not the local one:
            # it stamps odometry/gps with whatever frame its input carries, so
            # feeding it the odom-frame estimate made ekf_node_map fuse an
            # odom-frame quantity as an absolute map measurement - a feedback
            # loop that manufactured velocity with the wheels stationary.
            GroupAction([
                imu_enu_bridge(),
                Node(
                    package="robot_localization",
                    executable="navsat_transform_node",
                    name="navsat_transform_node",
                    namespace=NAMESPACE,
                    output="screen",
                    parameters=[GPS_CONFIG, {"use_sim_time": use_sim_time}],
                    remappings=[
                        # Unused while use_odometry_yaw is true: heading then
                        # comes from the global EKF's own output. Kept so
                        # flipping that parameter back needs no re-derivation.
                        ("imu", "sensors/imu_enu/data"),
                        # The RAW fix. The sim GPS reports all-zero covariance
                        # and robot_localization floors it itself; the removed
                        # covariance relay's invented 2.5 m value made the
                        # global EKF trust wheel velocity over GPS and drift
                        # 2.7 m while stationary.
                        ("gps/fix", "sensors/gps_0/fix"),
                        ("odometry/filtered", "platform/odom/filtered_map"),
                        ("gps/filtered", "gps/filtered"),
                        ("odometry/gps", "odometry/gps"),
                    ] + TF_REMAPS,
                ),
                ekf_node_map(GPS_CONFIG),
            ], condition=backend("gps")),

            # The RF comms bridge the rssi backend measures through.
            # Conditioned on the Node itself, so it adds no GroupAction.
            radio_bridge(),

            # The live marker view of that same measurement, same condition.
            rssi_viz(),

            # Stage 1b - RSSI. Same contract, no geodetic stage: trilateration
            # solves directly in map metres from surveyed tower coordinates, so
            # there is deliberately no navsat_transform_node here.
            GroupAction([
                imu_enu_bridge(),
                # The absolute position measurement, as a
                # PoseWithCovarianceStamped on <ns>/rssi/pose. No TF_REMAPS:
                # it neither broadcasts nor looks up TF, it only stamps its
                # pose `map`. `executable` is an absolute path because tools/
                # is not an ament package.
                Node(
                    executable=RSSI_NODE,
                    name="rssi_localization",
                    namespace=NAMESPACE,
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        # Explicit value_type: a LaunchConfiguration resolves
                        # to a STRING and the node declares both as doubles,
                        # so the raw substitution is rejected as a type
                        # mismatch at startup.
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rssi_rate_hz"),
                            value_type=float),
                        "position_covariance": ParameterValue(
                            LaunchConfiguration("rssi_position_covariance"),
                            value_type=float),
                    }],
                ),
                ekf_node_map(RSSI_CONFIG),
            ], condition=backend("rssi")),

            # Stage 2 - the prior map. Launched here, but brought up by the
            # single lifecycle manager below (ruling D4).
            Node(package="nav2_map_server", executable="map_server",
                 name="map_server", namespace=NAMESPACE, output="screen",
                 parameters=[MAP_CONFIG, {"use_sim_time": use_sim_time}]),

            # Stage 3 - nav2 proper: Clearpath's wrapper (ruling D2) around
            # nav2_bringup's node set (ruling D4), with our params file.
            GroupAction([
                PushRosNamespace(NAMESPACE),
                SetParameter("use_sim_time", use_sim_time),
                SetRemap("/" + NAMESPACE + "/odom",
                         "/" + NAMESPACE + "/platform/odom"),
                SetRemap("/tf", "/" + NAMESPACE + "/tf"),
                SetRemap("/tf_static", "/" + NAMESPACE + "/tf_static"),
                # No cmd_vel remap: velocity_smoother publishes
                # cmd_vel_smoothed and collision_monitor forwards it to
                # cmd_vel (stock Clearpath wiring). A remap bypassing the
                # monitor was needed only while the 2D lidar's 360 deg sweep
                # saw the robot's own sensor arch; the sweep is now +-135 deg.
                nav2_node("nav2_controller", "controller_server",
                          extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
                nav2_node("nav2_planner", "planner_server", "planner_server"),
                nav2_node("nav2_behaviors", "behavior_server", "behavior_server",
                          extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
                nav2_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
                nav2_node("nav2_waypoint_follower", "waypoint_follower",
                          "waypoint_follower"),
                nav2_node("nav2_velocity_smoother", "velocity_smoother",
                          "velocity_smoother",
                          extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
                nav2_node("nav2_collision_monitor", "collision_monitor",
                          "collision_monitor"),
            ]),

            # The one manager. Outside the Stage 3 group on purpose: it needs
            # the namespace but none of that group's topic remaps.
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_navigation", namespace=NAMESPACE,
                 output="screen", arguments=LOG_ARGS,
                 parameters=[{"use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "node_names": LIFECYCLE_NODES}]),
        ]

    # Fire once. `create` is expected to exit exactly once, but a handler that
    # can start a second nav2 stack is not a risk worth carrying.
    started = []

    # Stock robot: generators, platform/sensor services, control, spawn.
    # `rviz` is pinned false - this launcher's rviz is view_robot, not the
    # navigation view we start below. Returned from the GUI gate's exit
    # handler rather than listed directly, so it cannot fire while the GUI is
    # still loading park's meshes.
    #
    # No enclosing group here - a scope at EITHER level aborts the launch
    # before the robot spawns, so the condition rides on the gate process and
    # on the event handlers directly. See the docstring.
    robot_spawn = stock(pkg_clearpath_gz, "robot_spawn.launch.py", {
        "world": PARK_WORLD_NAME,
        "use_sim_time": use_sim_time,
        "rviz": "false",
        **{element: LaunchConfiguration(element) for element in PARK_SPAWN},
    })

    gui_ready = ExecuteProcess(
        cmd=["bash", "-c", GUI_READY_SCRIPT],
        name="gui_ready_gate",
        output="screen",
        condition=IfCondition(world_and_robot),
    )

    def on_gui_ready(event, context):
        """Spawn the robot, or fail loudly if the GUI never came up."""
        if event.returncode != 0:
            return [Shutdown(
                reason=f"GUI gate timed out: {GUI_READY_SERVICE} did not appear "
                       f"within {GUI_READY_DEADLINE_S:.0f} s, so the robot spawn "
                       f"would be missed by the GUI - not spawning")]
        return [robot_spawn]

    def on_world_ready(event, context):
        if started:
            return None
        started.append(True)
        return navigation_stages()

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("autostart", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("world", default_value=PARK_WORLD_PATH,
                              description="absolute path to the world, without "
                                          "the .sdf suffix"),
        DeclareLaunchArgument("localization", default_value="gps",
                              choices=LOCALIZATION_BACKENDS,
                              description="global localization backend "
                                          "(map -> odom source)"),
        DeclareLaunchArgument("rssi_position_covariance", default_value="0.25",
                              description="localization:=rssi only - measurement "
                                          "covariance of the trilaterated pose"),
        DeclareLaunchArgument("rssi_rate_hz", default_value="1.0",
                              description="localization:=rssi only - solve rate"),
        DeclareLaunchArgument("world_and_robot", default_value="true",
                              choices=["true", "false"],
                              description="include the stock world and robot "
                                          "launchers; false runs the navigation "
                                          "stages only, against a sim that is "
                                          "already up (scripts/sim.py)"),
        DeclareLaunchArgument("rviz", default_value="true",
                              choices=["true", "false"],
                              description="start the rviz view (the stock "
                                          "nav2 view; under localization:=rssi "
                                          "the same view with the RSSI "
                                          "MarkerArray display added, "
                                          "config/nav2_rssi.rviz)"),
    ] + [
        DeclareLaunchArgument(element, default_value=default,
                              description=f"{element} of the robot spawn pose "
                                          f"(park's authored value)")
        for element, default in PARK_SPAWN.items()
    ] + [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        # MUST precede the gz_sim include - see the ordering ruling above.
        AppendEnvironmentVariable("AMENT_PREFIX_PATH", GZ_PREFIX, prepend=True),

        # Stock world.
        stock(pkg_clearpath_gz, "gz_sim.launch.py", {
            "world": world,
            "use_sim_time": use_sim_time,
        }, condition=IfCondition(world_and_robot)),

        # Our `rviz` decision, captured under a name no include writes. The
        # robot_spawn include below is unscoped and pins `rviz` false, which
        # therefore leaks into this scope - reading `rviz` after it would
        # always yield false. MUST stay ahead of that include.
        SetLaunchConfiguration("nav_rviz", LaunchConfiguration("rviz")),

        # The GUI gate: poll for the last GUI plugin's service under a hard
        # deadline, and hang the robot spawn off its exit. See the docstring.
        gui_ready,
        RegisterEventHandler(
            OnProcessExit(target_action=gui_ready, on_exit=on_gui_ready),
            condition=IfCondition(world_and_robot),
        ),

        # The readiness gate: Stages 1-3 start when `create` returns, i.e.
        # when Gazebo has answered the spawn request and park is stepping.
        RegisterEventHandler(
            OnProcessExit(
                target_action=_is_robot_spawn_process,
                on_exit=on_world_ready,
            ),
            condition=IfCondition(world_and_robot),
        ),

        # world_and_robot:=false - the world is already up and stepping, so
        # there is nothing to wait for and no `create` process to wait on.
        # This is the seam scripts/sim.py uses: it owns phases 1-5 (clean,
        # world, controllers, robot, extras) and calls this file for the
        # navigation stages alone, so the stages live here once and only once.
        GroupAction(navigation_stages(),
                    condition=UnlessCondition(world_and_robot)),

        # The rviz view: always the stock launcher, never a substitute for
        # it. Only the CONFIG it is handed varies by backend - `rssi` gets
        # config/nav2_rssi.rviz (stock nav2.rviz plus the RSSI MarkerArray),
        # anything else gets stock's own default file name. Standing rule:
        # add to the stock chain, never switch away from it. An earlier
        # revision started a bare rviz2 Node under `rssi` instead of this
        # include and silently lost three stock tools with it, GoalTool - the
        # click-to-set-goal button - among them.
        #
        # Two SetLaunchConfiguration actions rather than one include per
        # backend, so there is exactly one rviz action in this file: they are
        # mutually exclusive by construction (EqualsSubstitution on
        # `localization` and its negation), so `rviz_config` is written
        # exactly once and `rviz:=true` starts exactly one window,
        # `rviz:=false` none. Conditions ride on the actions themselves; no
        # GroupAction.
        SetLaunchConfiguration(
            "rviz_config", NAV2_RSSI_RVIZ,
            condition=IfCondition(EqualsSubstitution(localization, "rssi"))),
        SetLaunchConfiguration(
            # view_navigation.launch.py:62 declares this same string as its
            # default, so this branch is stock behaviour written out.
            "rviz_config", "nav2.rviz",
            condition=UnlessCondition(
                EqualsSubstitution(localization, "rssi"))),

        # It does its own PushRosNamespace and /tf remaps, so it needs no help
        # from here beyond the namespace. Not gated on readiness: rviz is
        # content to wait for topics and shows the world coming up.
        stock(pkg_clearpath_viz, "view_navigation.launch.py", {
            "namespace": NAMESPACE,
            "use_sim_time": use_sim_time,
            "config": LaunchConfiguration("rviz_config"),
        }, condition=IfCondition(LaunchConfiguration("nav_rviz"))),
    ])
