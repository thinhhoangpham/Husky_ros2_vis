import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
NS='/a200_0000'
def rpy(q):
    r=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x**2+q.y**2))
    p=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
    return math.degrees(r), math.degrees(p)
class C(Node):
    def __init__(s):
        super().__init__('terrain'); s.imu=None; s.odom=None
        s.create_subscription(Imu,NS+'/sensors/imu_0/data',lambda m:setattr(s,'imu',m),qos_profile_sensor_data)
        s.create_subscription(Odometry,NS+'/platform/odom',lambda m:setattr(s,'odom',m),qos_profile_sensor_data)
        s.pub=s.create_publisher(TwistStamped,NS+'/cmd_vel',10)
    def drive(s,v,sec,log=None):
        t=time.time(); last=0
        while time.time()-t<sec:
            m=TwistStamped(); m.header.stamp=s.get_clock().now().to_msg(); m.twist.linear.x=v
            s.pub.publish(m); rclpy.spin_once(s,timeout_sec=0.02)
            if log is not None and time.time()-last>1.0 and s.imu and s.odom:
                last=time.time(); r,p=rpy(s.imu.orientation)
                log.append((s.odom.pose.pose.position.x, s.odom.pose.pose.position.z, r, p))
    def stop(s,sec=1.5):
        t=time.time()
        while time.time()-t<sec:
            m=TwistStamped(); m.header.stamp=s.get_clock().now().to_msg()
            s.pub.publish(m); rclpy.spin_once(s,timeout_sec=0.02)
rclpy.init(); c=C()
t=time.time()
while time.time()-t<10 and (c.imu is None or c.odom is None): rclpy.spin_once(c,timeout_sec=0.1)
c.stop(2.0)
r,p=rpy(c.imu.orientation)
print(f"FLAT GROUND     : roll={r:+6.2f}  pitch={p:+6.2f}  (odom x={c.odom.pose.pose.position.x:+.2f})")
print("\nDriving forward onto the 15 deg ramp...\n")
print(f"{'odom x':>8}{'odom z':>9}{'roll':>9}{'pitch':>9}")
log=[]
c.drive(0.7, 16.0, log)
c.stop(2.5)
for x,z,r,p in log: print(f"{x:8.2f}{z:9.2f}{r:9.2f}{p:9.2f}")
r,p=rpy(c.imu.orientation)
print(f"\nON THE RAMP     : roll={r:+6.2f}  pitch={p:+6.2f}   (ramp is 15.00 deg)")
print(f"error vs terrain slope: {abs(abs(p)-15.0):.2f} deg")
rclpy.shutdown()
