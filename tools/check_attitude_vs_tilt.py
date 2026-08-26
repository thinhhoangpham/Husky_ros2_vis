import math, time, subprocess, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
NS='/a200_0000'

def quat(r,p,y):
    cr,sr=math.cos(r/2),math.sin(r/2); cp,sp=math.cos(p/2),math.sin(p/2); cy,sy=math.cos(y/2),math.sin(y/2)
    return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)  # w x y z

def teleport(r,p,y):
    w,x,yy,z = quat(r,p,y)
    subprocess.run(['gz','service','-s','/world/warehouse/set_pose','--reqtype','gz.msgs.Pose',
        '--reptype','gz.msgs.Boolean','--timeout','500','--req',
        f'name: "a200_0000/robot", position: {{x: 2, y: 0, z: 0.35}}, '
        f'orientation: {{w: {w}, x: {x}, y: {yy}, z: {z}}}'], capture_output=True)

def rpy(q):
    r=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x**2+q.y**2))
    p=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
    y=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
    return map(math.degrees,(r,p,y))

class C(Node):
    def __init__(s):
        super().__init__('att'); s.imu=None; s.mag=None; s.odom=None
        s.create_subscription(Imu,NS+'/sensors/imu_0/data',lambda m:setattr(s,'imu',m),qos_profile_sensor_data)
        s.create_subscription(MagneticField,NS+'/sensors/compass_0/mag',lambda m:setattr(s,'mag',m),qos_profile_sensor_data)
        s.create_subscription(Odometry,NS+'/platform/odom',lambda m:setattr(s,'odom',m),qos_profile_sensor_data)
    def hold(s, r,p,y, sec=2.5):
        t=time.time()
        while time.time()-t<sec:
            teleport(r,p,y)
            for _ in range(4): rclpy.spin_once(s,timeout_sec=0.02)
        return s.imu, s.mag

rclpy.init(); c=C()
t=time.time()
while time.time()-t<8 and (c.imu is None or c.mag is None): rclpy.spin_once(c,timeout_sec=0.1)

print(f"{'commanded pose':<26}{'IMU roll/pitch/yaw':<30}{'gyro.z':>9}   {'compass heading':>16}")
print("-"*88)
for name,(r,p,y) in [("level, yaw   0",(0,0,0)), ("level, yaw  90",(0,0,90)),
                     ("ROLL 30, yaw 0",(30,0,0)), ("PITCH 25, yaw 0",(0,25,0))]:
    imu,mag = c.hold(math.radians(r),math.radians(p),math.radians(y))
    ir,ip,iy = rpy(imu.orientation)
    f=mag.magnetic_field
    head=math.degrees(math.atan2(-f.y,f.x))
    print(f"{name:<26}{ir:+7.1f} {ip:+7.1f} {iy:+7.1f}    {imu.angular_velocity.z:+8.3f}   {head:+15.1f}")

print("\n--- what each sensor actually reports ---")
imu,mag=c.hold(0,0,0)
print("IMU     : orientation(quat) + angular_velocity(gyro) + linear_acceleration(accel)")
print("compass : magnetic_field only ->", f"x={mag.magnetic_field.x:+.4f} y={mag.magnetic_field.y:+.4f} z={mag.magnetic_field.z:+.4f}")
rclpy.shutdown()
