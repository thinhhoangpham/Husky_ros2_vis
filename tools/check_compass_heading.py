import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
NS='/a200_0000'
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
class C(Node):
    def __init__(s):
        super().__init__('compass_check')
        s.mag=None; s.odom=None
        s.create_subscription(MagneticField, NS+'/sensors/compass_0/mag', lambda m:setattr(s,'mag',m), qos_profile_sensor_data)
        s.create_subscription(Odometry, NS+'/platform/odom', lambda m:setattr(s,'odom',m), qos_profile_sensor_data)
        s.pub=s.create_publisher(TwistStamped, NS+'/cmd_vel', 10)
    def cmd(s,lx,az,sec):
        t=time.time()
        while time.time()-t<sec:
            m=TwistStamped(); m.header.stamp=s.get_clock().now().to_msg()
            m.twist.linear.x=lx; m.twist.angular.z=az
            s.pub.publish(m); rclpy.spin_once(s,timeout_sec=0.02)
    def read(s):
        t=time.time()
        while time.time()-t<3 and s.mag is None: rclpy.spin_once(s,timeout_sec=0.05)
        s.cmd(0.0,0.0,1.0)
        f=s.mag.magnetic_field
        return math.degrees(math.atan2(-f.y,f.x)), math.degrees(yaw(s.odom.pose.pose.orientation)), f
rclpy.init(); c=C()
t=time.time()
while time.time()-t<8 and (c.mag is None or c.odom is None): rclpy.spin_once(c,timeout_sec=0.1)
if c.mag is None: print("NO COMPASS DATA ON ROS"); raise SystemExit(1)
h0,y0,f=c.read()
print("frame_id            :", c.mag.header.frame_id)
print("field [T]           : x=%+.4f y=%+.4f z=%+.4f  |B|=%.4f" % (f.x,f.y,f.z,math.sqrt(f.x**2+f.y**2+f.z**2)))
print("compass heading     : %+.1f deg   (odom yaw %+.1f)" % (h0,y0))
c.cmd(0.0,0.5,3.2); c.cmd(0.0,0.0,1.5)
h1,y1,_=c.read()
dh=(h1-h0+540)%360-180; dy=(y1-y0+540)%360-180
print("\nafter CCW rotation  : compass %+.1f -> %+.1f  (delta %+.1f deg)" % (h0,h1,dh))
print("                      odom    %+.1f -> %+.1f  (delta %+.1f deg)" % (y0,y1,dy))
print("agreement           : %.1f deg error" % abs(dh-dy))
rclpy.shutdown()
