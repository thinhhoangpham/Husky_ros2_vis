#!/usr/bin/env python3
"""Relay /a200_0000/sensors/gps_0/fix -> .../fix_cov with a stamped covariance.

Gazebo's navsat sensor plugin emits sensor_msgs/NavSatFix with
position_covariance_type COVARIANCE_TYPE_UNKNOWN and an all-zero matrix (no
noise model). A zero covariance matrix is singular, so the EKF's Kalman gain
for the GPS update is numerically undefined - this was the confirmed root
cause of ekf_node_map's unbounded, velocity-free divergence (Task 4 fix
session 2).

robot_localization has no per-sensor covariance override parameter, and the
zero covariance originates in the generated ~/clearpath/sensors/config/
gps_0.yaml bridge config, which must never be edited (regenerated at every
launch). So this relay sits between the bridge and navsat_transform_node,
stamping a plausible covariance so the EKF update is well-posed.

This node does no filtering, no rate change, no reprojection - it copies the
message and overwrites position_covariance / position_covariance_type only.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

# INVENTED sensor characteristic, not measured: the sim GPS reports no noise
# model at all (position_covariance_type 0, all zeros), so there is nothing
# to derive real numbers from. These stand in for a consumer-grade GPS
# (~2-3 m 1-sigma horizontal, worse vertical) purely so the EKF's Kalman
# gain for this update is well-posed. Diagonal-only (COVARIANCE_TYPE_DIAGONAL_KNOWN).
GPS_STDDEV_XY_M = 2.5   # metres, 1-sigma, horizontal (east/north)
GPS_STDDEV_Z_M = 5.0    # metres, 1-sigma, vertical - GPS altitude is worse
GPS_VARIANCE_XY = GPS_STDDEV_XY_M ** 2
GPS_VARIANCE_Z = GPS_STDDEV_Z_M ** 2

IN_TOPIC = "sensors/gps_0/fix"
OUT_TOPIC = "sensors/gps_0/fix_cov"


class GpsCovarianceRelay(Node):
    def __init__(self) -> None:
        super().__init__("gps_covariance_relay")
        self.pub = self.create_publisher(NavSatFix, OUT_TOPIC, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            NavSatFix, IN_TOPIC, self.callback, qos_profile_sensor_data
        )

    def callback(self, msg: NavSatFix) -> None:
        msg.position_covariance = [
            GPS_VARIANCE_XY, 0.0, 0.0,
            0.0, GPS_VARIANCE_XY, 0.0,
            0.0, 0.0, GPS_VARIANCE_Z,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GpsCovarianceRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
