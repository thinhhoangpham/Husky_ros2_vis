#!/usr/bin/env python3
"""Relay /a200_0000/sensors/imu_enu/data -> .../imu_map/data, ENU yaw -> map yaw.

urdf/imu_enu.urdf.xacro adds a second gz-sim IMU declaring
<orientation_reference_frame><localization>ENU</localization>, so its
orientation is referenced to the world ENU frame rather than to the sensor's
spawn pose (which is what stock imu_0 reports - measured yaw 0.0000 at world
yaw 149.72 deg in park, the confirmed root cause of ekf_node_map carrying a
constant -149.7 deg map -> odom rotation error).

park.sdf declares heading_deg 90, so the Gazebo world/map frame is rotated
+pi/2 from ENU. Measured on an ENU-localized IMU spawned in park: ENU yaw
59.73 deg at world yaw 149.72 deg, i.e. map yaw = ENU yaw + pi/2 exactly.
This node applies that rotation so ekf_node_map can fuse yaw as an absolute
map-frame heading.

navsat_transform_node does NOT consume this topic - it expects ENU yaw and
adds yaw_offset itself, so it stays on the raw sensors/imu_enu/data.

The rotation is a change of the orientation's REFERENCE frame, so it composes
on the world side: q_out = q_rot (x) q_in (left multiplication).
angular_velocity and linear_acceleration are body-frame quantities and are
unaffected by a reference-frame yaw, so they are passed through untouched.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

IN_TOPIC = "sensors/imu_enu/data"
OUT_TOPIC = "sensors/imu_map/data"

# ENU -> world/map: +pi/2 about z (park.sdf heading_deg 90).
HALF_ANGLE = math.pi / 4.0
ROT_Z = math.sin(HALF_ANGLE)
ROT_W = math.cos(HALF_ANGLE)

# INVENTED sensor characteristic, not measured: gz-sim's IMU publishes an
# all-zero orientation covariance (no noise model). A zero matrix is singular,
# so the EKF's Kalman gain for the yaw update would be undefined - the same
# trap the sim GPS had (tools/gps_covariance_relay.py). 1e-4 rad^2 is 0.57 deg
# 1-sigma: tight, because the sim orientation is physics ground truth, but
# nonzero so the update is well-posed.
DEFAULT_ORIENTATION_VARIANCE = 1e-4


class ImuMapRelay(Node):
    def __init__(self) -> None:
        super().__init__("imu_map_relay")
        self.pub = self.create_publisher(Imu, OUT_TOPIC, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Imu, IN_TOPIC, self.callback, qos_profile_sensor_data
        )

    def callback(self, msg: Imu) -> None:
        # Snapshot first: msg.orientation is a reference, so writing back
        # component-by-component would feed already-rotated values into the
        # remaining terms.
        qx, qy, qz, qw = (
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        )
        # q_out = q_rot (x) q_in, with q_rot = (0, 0, ROT_Z, ROT_W).
        msg.orientation.w = ROT_W * qw - ROT_Z * qz
        msg.orientation.x = ROT_W * qx - ROT_Z * qy
        msg.orientation.y = ROT_W * qy + ROT_Z * qx
        msg.orientation.z = ROT_W * qz + ROT_Z * qw

        if not any(msg.orientation_covariance):
            msg.orientation_covariance = [
                DEFAULT_ORIENTATION_VARIANCE, 0.0, 0.0,
                0.0, DEFAULT_ORIENTATION_VARIANCE, 0.0,
                0.0, 0.0, DEFAULT_ORIENTATION_VARIANCE,
            ]
        elif msg.orientation_covariance[0] == 0.0:
            msg.orientation_covariance[0] = DEFAULT_ORIENTATION_VARIANCE

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ImuMapRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
