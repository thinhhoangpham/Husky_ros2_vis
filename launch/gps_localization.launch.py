"""Global (GPS) localization for park: publishes map -> odom.

Runs alongside, and never replaces, the Clearpath-generated ekf_node that
owns odom -> base_link.

Ruling D3: the Clearpath stack remaps /tf and /tf_static to namespaced
topics throughout (clearpath_control/control.launch.py,
clearpath_control/localization.launch.py, the generated
platform-service.launch.py). Both nodes below must remap /tf and
/tf_static to the namespaced topics too, or ekf_node_map's map -> odom
publish lands on a topic nothing else in this stack reads.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG = "/home/thinhpham/Documents/Husky_viz/config/gps_localization.yaml"
IMU_RELAY_SCRIPT = "/home/thinhpham/Documents/Husky_viz/tools/imu_map_relay.py"
NAMESPACE = "a200_0000"

TF_REMAPS = [
    ("/tf", "/a200_0000/tf"),
    ("/tf_static", "/a200_0000/tf_static"),
]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Rotates the ENU-referenced imu_enu orientation into the Gazebo
        # world/map frame (+pi/2 about z, park.sdf heading_deg 90) so
        # ekf_node_map can fuse yaw as an absolute map-frame heading.
        ExecuteProcess(
            cmd=[
                "python3", IMU_RELAY_SCRIPT,
                "--ros-args",
                "-r", f"__ns:=/{NAMESPACE}",
            ],
            name="imu_map_relay",
            output="screen",
        ),

        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            namespace=NAMESPACE,
            output="screen",
            parameters=[CONFIG, {"use_sim_time": use_sim_time}],
            remappings=[
                # The RAW ENU topic, not sensors/imu_map/data:
                # navsat_transform expects ENU yaw and applies yaw_offset
                # itself. Stock imu_0 is spawn-relative (yaw 0.0000 at world
                # yaw 149.72 deg in park) and was never a world heading.
                ("imu", "sensors/imu_enu/data"),
                # The RAW fix, matching the ROS 1 reference (which had no
                # covariance relay): the sim GPS reports all-zero covariance
                # and robot_localization floors it to a small value itself.
                # gps_covariance_relay was removed 2026-08-26 - its invented
                # 2.5 m covariance made the global EKF trust wheel velocity
                # over GPS, drifting 2.7 m while the robot was stationary.
                ("gps/fix", "sensors/gps_0/fix"),
                # Bug found by the coordinator (Task 4 fix session 4): this
                # MUST be the global filter's own output (filtered_map), not
                # the local filter's (filtered, odom-frame). navsat_transform
                # emits odometry/gps stamped with whatever frame this input
                # carries; feeding it the odom-frame local estimate made
                # odometry/gps an odom-frame quantity that ekf_node_map then
                # fused as an absolute MAP-frame measurement - a positive
                # feedback loop (the filter's own map -> odom output shifted
                # the frame of its next GPS input) that manufactured apparent
                # velocity with the wheels stationary and no IMU fused. This
                # is the canonical robot_localization dual-EKF + navsat
                # topology: navsat_transform consumes the GLOBAL filter's
                # estimate, not the local one.
                ("odometry/filtered", "platform/odom/filtered_map"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
            ] + TF_REMAPS,
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node_map",
            namespace=NAMESPACE,
            output="screen",
            parameters=[CONFIG, {"use_sim_time": use_sim_time}],
            remappings=[
                ("odometry/filtered", "platform/odom/filtered_map"),
            ] + TF_REMAPS,
        ),
    ])
