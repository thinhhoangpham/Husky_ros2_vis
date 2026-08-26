import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
NS='/a200_0000'
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
class C(Node):
    def __init__(s):
        super().__init__('gyro_check'); s.imu=None
        s.create_subscription(Imu,NS+'/sensors/imu_0/data',lambda m:setattr(s,'imu',m),qos_profile_sensor_data)
        s.pub=s.create_publisher(TwistStamped,NS+'/cmd_vel',10)
    def spin(s,sec):
        t=time.time()
        while time.time()-t<sec: rclpy.spin_once(s,timeout_sec=0.02)
rclpy.init(); c=C()
t=time.time()
while time.time()-t<8 and c.imu is None: rclpy.spin_once(c,timeout_sec=0.1)

c.spin(1.5)
print("AT REST (robot stationary, but NOT at heading zero):")
print(f"  gyro.z          = {c.imu.angular_velocity.z:+.4f} rad/s   <- rate: zero")
print(f"  orientation yaw = {math.degrees(yaw(c.imu.orientation)):+.1f} deg      <- angle: not zero")

# rotate, integrating gyro.z, and compare against the orientation field
y0 = yaw(c.imu.orientation); integ = 0.0; last = time.time(); peak = 0.0
t = time.time()
while time.time()-t < 4.0:
    m=TwistStamped(); m.header.stamp=c.get_clock().now().to_msg(); m.twist.angular.z=0.6
    c.pub.publish(m); rclpy.spin_once(c,timeout_sec=0.02)
    now=time.time(); gz=c.imu.angular_velocity.z
    integ += gz*(now-last); last=now; peak=max(peak,gz)
for _ in range(20):
    m=TwistStamped(); m.header.stamp=c.get_clock().now().to_msg(); c.pub.publish(m); rclpy.spin_once(c,timeout_sec=0.02)
c.spin(1.5)
y1 = yaw(c.imu.orientation)
d_orient = math.degrees((y1-y0+math.pi)%(2*math.pi)-math.pi)
print(f"\nDURING A 4 s ROTATION (cmd +0.6 rad/s):")
print(f"  peak gyro.z              = {peak:+.3f} rad/s")
print(f"  integral of gyro.z       = {math.degrees(integ):+.1f} deg   <- angle you must COMPUTE")
print(f"  delta of orientation yaw = {d_orient:+.1f} deg   <- angle reported DIRECTLY")
print(f"  difference               = {abs(math.degrees(integ)-d_orient):.1f} deg")
rclpy.shutdown()
