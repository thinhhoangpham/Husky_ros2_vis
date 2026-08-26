import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from nav_msgs.msg import Odometry
NS='/a200_0000'
T=[('wheel odom',      Odometry,   NS+'/platform/odom'),
   ('odom (EKF fused)',Odometry,   NS+'/platform/odom/filtered'),
   ('imu_0',           Imu,        NS+'/sensors/imu_0/data'),
   ('lidar2d (gpu)',   LaserScan,  NS+'/sensors/lidar2d_0/scan'),
   ('lidar3d (gpu)',   PointCloud2,NS+'/sensors/lidar3d_0/points')]
rclpy.init(); n=Node('dc'); c={k:0 for k,_,_ in T}
for name,typ,top in T:
    n.create_subscription(typ, top, (lambda k: (lambda m: c.__setitem__(k,c[k]+1)))(name), qos_profile_sensor_data)
t=time.time()
while time.time()-t<6: rclpy.spin_once(n,timeout_sec=0.05)
print(f"{'stream':<20}{'Hz':>7}   status")
for name,_,_ in T:
    hz=c[name]/6.0
    print(f"{name:<20}{hz:>7.1f}   {'OK' if hz>0.5 else 'NO DATA'}")
rclpy.shutdown()
