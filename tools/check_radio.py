import time, subprocess, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Dataframe

def teleport(x):
    subprocess.run(['gz','service','-s','/world/warehouse/set_pose',
        '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000',
        '--req', f'name: "a200_0000/robot", position: {{x: {x}, y: 0, z: 0.3}}, orientation: {{w: 1}}'],
        capture_output=True)

class C(Node):
    def __init__(s):
        super().__init__('rf_check')
        q=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        s.rx_base=[]; s.rx_husky=[]
        s.create_subscription(Dataframe,'/base_station/rx', lambda m:s.rx_base.append(m), q)
        s.create_subscription(Dataframe,'/husky/rx',        lambda m:s.rx_husky.append(m), q)
        s.pub=s.create_publisher(Dataframe,'/broker/msgs', q)
    def spin(s,sec):
        t=time.time()
        while time.time()-t<sec: rclpy.spin_once(s,timeout_sec=0.05)
    def send(s,src,dst,payload):
        m=Dataframe(); m.src_address=src; m.dst_address=dst
        m.data=list(payload.encode()); s.pub.publish(m); s.spin(1.5)

rclpy.init(); c=C(); c.spin(2.0)

print("=== close range (robot at spawn, base_station at origin) ===")
teleport(2.0); c.spin(2.0)
n0=len(c.rx_base); c.send('husky','base_station','hello from husky')
got=c.rx_base[n0:]
print(f"  husky -> base_station : {len(got)} packet(s)")
for m in got:
    print(f"    payload={bytes(m.data).decode()!r}  rssi={m.rssi:.1f} dBm  src={m.src_address}")

n0=len(c.rx_husky); c.send('base_station','husky','ack from base')
got=c.rx_husky[n0:]
print(f"  base_station -> husky : {len(got)} packet(s)")
for m in got:
    print(f"    payload={bytes(m.data).decode()!r}  rssi={m.rssi:.1f} dBm  src={m.src_address}")

for dist in (30.0, 80.0, 300.0):
    print(f"=== robot teleported to x={dist:.0f} m (max_range=50 m) ===")
    teleport(dist); c.spin(2.5)
    n0=len(c.rx_base); c.send('husky','base_station',f'ping at {dist:.0f}m')
    got=c.rx_base[n0:]
    if got: print(f"  delivered, rssi={got[-1].rssi:.1f} dBm")
    else:   print("  no packet delivered (out of range)")
teleport(2.0)
rclpy.shutdown()
