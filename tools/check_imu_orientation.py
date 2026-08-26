import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

NS='/a200_0000'
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))

class C(Node):
    def __init__(s):
        super().__init__('imu_check')
        s.raw=None; s.filt=None; s.odom=None
        s.create_subscription(Imu, NS+'/sensors/imu_0/data', lambda m:setattr(s,'raw',m), qos_profile_sensor_data)
        s.create_subscription(Imu, NS+'/sensors/imu_0/data',     lambda m:setattr(s,'filt',m), qos_profile_sensor_data)
        s.create_subscription(Odometry, NS+'/platform/odom',     lambda m:setattr(s,'odom',m), qos_profile_sensor_data)
        s.pub=s.create_publisher(TwistStamped, NS+'/cmd_vel', 10)
    def cmd(s, lx, az, sec, sample=None):
        t=time.time()
        while time.time()-t<sec:
            m=TwistStamped(); m.header.stamp=s.get_clock().now().to_msg()
            m.twist.linear.x=lx; m.twist.angular.z=az
            s.pub.publish(m); rclpy.spin_once(s, timeout_sec=0.02)
            if sample is not None and s.raw: sample.append(s.raw)
    def settle(s, sec):
        s.cmd(0.0,0.0,sec)

rclpy.init(); c=C()
t=time.time()
while time.time()-t<5.0 and c.raw is None: rclpy.spin_once(c, timeout_sec=0.1)
if c.raw is None:
    print("NO IMU DATA RECEIVED"); rclpy.shutdown(); raise SystemExit(1)

c.settle(1.0)
a=c.raw.linear_acceleration; g=c.raw.angular_velocity
print("frame_id             :", c.raw.header.frame_id)
print("REST accel  [m/s^2]  : x=%+.3f y=%+.3f z=%+.3f  |a|=%.3f" % (a.x,a.y,a.z,math.sqrt(a.x**2+a.y**2+a.z**2)))
print("REST gyro   [rad/s]  : x=%+.4f y=%+.4f z=%+.4f" % (g.x,g.y,g.z))
if c.filt:
    q=c.filt.orientation
    r=math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x**2+q.y**2)))
    p=math.degrees(math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x)))))
    print("REST orient [deg]    : roll=%+.2f pitch=%+.2f yaw=%+.2f" % (r,p,math.degrees(yaw(q))))

y0=math.degrees(yaw(c.odom.pose.pose.orientation)) if c.odom else float('nan')
s=[]; c.cmd(0.0, 0.6, 3.0, s); c.settle(1.0)
y1=math.degrees(yaw(c.odom.pose.pose.orientation)) if c.odom else float('nan')
gz=[m.angular_velocity.z for m in s]
print("\nCCW cmd +0.6 rad/s   : mean gyro.z=%+.3f   odom yaw %+.1f -> %+.1f deg" % (sum(gz)/len(gz), y0, y1))

s=[]; c.cmd(0.8, 0.0, 1.2, s); c.settle(1.0)
ax=[m.linear_acceleration.x for m in s]
print("FWD cmd +0.8 m/s     : peak accel.x=%+.3f m/s^2" % max(ax))
rclpy.shutdown()
