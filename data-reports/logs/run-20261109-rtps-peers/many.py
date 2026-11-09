import rclpy,sys,time
from rclpy.context import Context
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
n=int(sys.argv[1]); dur=float(sys.argv[2]); nodes=[]
c0=Context(); rclpy.init(context=c0); probe=rclpy.create_node("claude_odom_probe",context=c0); cnt=[0]
probe.create_subscription(Odometry,"/base/odom",lambda m:cnt.__setitem__(0,cnt[0]+1),QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE))
ex=SingleThreadedExecutor(context=c0); ex.add_node(probe)
def spin(s):
    e=time.time()+s
    while time.time()<e: ex.spin_once(timeout_sec=0.05)
spin(6); cnt[0]=0; spin(10); print("baseline odom/10s",cnt[0],flush=True)
for i in range(n):
    c=Context(); rclpy.init(context=c); nodes.append((c,rclpy.create_node("claude_many_%d"%i,context=c)))
t0=time.time(); print("up",n,"participants",flush=True)
while time.time()-t0<dur:
    cnt[0]=0; spin(10); print("t+%2.0fs odom/10s %d"%(time.time()-t0,cnt[0]),flush=True)
for c,nd in nodes: nd.destroy_node(); rclpy.shutdown(context=c)
cnt[0]=0; spin(10); print("after removal: odom/10s",cnt[0],flush=True)
