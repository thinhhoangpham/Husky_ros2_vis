import math, time, subprocess, rclpy
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
def teleport(x,y,z,pitch):
    p=pitch/2; w,qy=math.cos(p),math.sin(p)
    subprocess.run(['gz','service','-s','/world/warehouse/set_pose','--reqtype','gz.msgs.Pose',
      '--reptype','gz.msgs.Boolean','--timeout','800','--req',
      f'name: "a200_0000/robot", position: {{x: {x}, y: {y}, z: {z}}}, '
      f'orientation: {{w: {w}, x: 0, y: {qy}, z: 0}}'], capture_output=True)
class C(Node):
    def __init__(s):
        super().__init__('t2'); s.imu=None; s.odom=None
        s.create_subscription(Imu,NS+'/sensors/imu_0/data',lambda m:setattr(s,'imu',m),qos_profile_sensor_data)
        s.create_subscription(Odometry,NS+'/platform/odom',lambda m:setattr(s,'odom',m),qos_profile_sensor_data)
        s.pub=s.create_publisher(TwistStamped,NS+'/cmd_vel',10)
    def settle(s,sec):
        t=time.time()
        while time.time()-t<sec:
            m=TwistStamped(); m.header.stamp=s.get_clock().now().to_msg()
            s.pub.publish(m); rclpy.spin_once(s,timeout_sec=0.02)
rclpy.init(); c=C()
t=time.time()
while time.time()-t<10 and c.imu is None: rclpy.spin_once(c,timeout_sec=0.1)

SLOPE=15.0
# place the robot on flat floor
teleport(0.0, 0.0, 0.35, 0.0); c.settle(3.0)
r,p=rpy(c.imu.orientation)
print(f"FLAT FLOOR (slope   0 deg): IMU roll={r:+6.2f}  pitch={p:+6.2f}")

# place it on the ramp surface, already aligned to the slope, and let it settle
# ramp: centre x=8, z=0.6, pitched -15 deg. surface height at x: 0.6 + (x-8)*tan(15) + 0.1/cos(15)
x=8.0
zs = 0.6 + (x-8.0)*math.tan(math.radians(SLOPE)) + 0.1/math.cos(math.radians(SLOPE))
teleport(x, 0.0, zs+0.25, -math.radians(SLOPE)); c.settle(4.0)
r,p=rpy(c.imu.orientation)
print(f"ON {SLOPE:.0f} DEG RAMP        : IMU roll={r:+6.2f}  pitch={p:+6.2f}")
print(f"\nIMU pitch vs terrain slope : {abs(abs(p)-SLOPE):.2f} deg error")
print(f"Robot tilt RELATIVE TO RAMP: 0 deg (it sits flush) - but IMU still reports {abs(p):.1f}")
teleport(0.0,0.0,0.35,0.0)
rclpy.shutdown()
