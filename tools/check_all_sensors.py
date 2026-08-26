import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, Image, MagneticField
from nav_msgs.msg import Odometry
NS='/a200_0000'
T=[('camera_0 RGB',   Image,        NS+'/sensors/camera_0/color/image'),
   ('camera_0 depth', Image,        NS+'/sensors/camera_0/depth/image'),
   ('lidar2d (gpu)',  LaserScan,    NS+'/sensors/lidar2d_0/scan'),
   ('lidar3d (gpu)',  PointCloud2,  NS+'/sensors/lidar3d_0/points'),
   ('imu_0',          Imu,          NS+'/sensors/imu_0/data'),
   ('compass_0',      MagneticField,NS+'/sensors/compass_0/mag'),
   ('odom',           Odometry,     NS+'/platform/odom')]
rclpy.init(); n=Node('final_check'); c={k:0 for k,_,_ in T}
for name,typ,top in T:
    n.create_subscription(typ, top, (lambda k: (lambda m: c.__setitem__(k, c[k]+1)))(name), qos_profile_sensor_data)
t=time.time()
while time.time()-t<6.0: rclpy.spin_once(n, timeout_sec=0.05)
print(f"{'sensor':<18}{'Hz':>8}   status")
for name,_,_ in T:
    hz=c[name]/6.0
    print(f"{name:<18}{hz:>8.1f}   {'OK' if hz>0.5 else 'NO DATA'}")
rclpy.shutdown()
