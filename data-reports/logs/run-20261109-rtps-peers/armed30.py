"""Armed zero-twist hold with N extra ROS 2 participants. Logs JSONL. Never commands motion."""
import sys, time, json, threading, socket, struct, zlib, re
sys.path.insert(0, "/home/snorlax/mib_debug_claude/racer/tools")
import rclpy, eth_link as el
from rclpy.context import Context
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from rammp_base_interfaces.msg import MibStatus, DriveCommand, TwistSetpoint

N = int(sys.argv[1]); OUT = open(sys.argv[2], "w", buffering=1)
T_ARM, T_ADD, T_DEL, T_DIS, T_END = 10, 30, 90, 110, 120
WHEELS = {"left": "192.168.4.4", "right": "192.168.4.3"}
t0 = time.time(); lock = threading.Lock()
def log(kind, **kw):
    with lock: OUT.write(json.dumps(dict(t=round(time.time() - t0, 3), kind=kind, **kw)) + "\n")

be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
c0 = Context(); rclpy.init(context=c0); node = rclpy.create_node("claude_armed_test", context=c0)
st = {"state": None}; odom = [0]
node.create_subscription(MibStatus, "/rammp/mib/status", lambda m: st.__setitem__("state", m.system_state), be)
node.create_subscription(Odometry, "/base/odom", lambda m: odom.__setitem__(0, odom[0] + 1), rel)
dc = node.create_publisher(DriveCommand, "/rammp/drive_command", be)
tw = node.create_publisher(TwistSetpoint, "/base/setpoint/twist", be)
from rclpy.executors import SingleThreadedExecutor
ex = SingleThreadedExecutor(context=c0); ex.add_node(node)
def cmd(req):
    m = DriveCommand(); m.request = req; m.profile = DriveCommand.PROFILE_LOW; m.push_assist = False; dc.publish(m)

# --- MIB status + wheel consoles, on their own threads
stop = threading.Event(); faulted = {"why": None}
def mib_poll():
    def frame(mod, typ): f = struct.pack("<HBBBI", 0x4F54, 0x10, mod, typ, 0); return f + struct.pack("<I", zlib.crc32(f))
    while not stop.is_set():
        try:
            c = socket.create_connection(("192.168.4.1", 5555), timeout=2); c.sendall(frame(0x40, 0x09)); b = b""; end = time.time() + 2; js = None
            while time.time() < end and js is None:
                try: b += c.recv(16384)
                except socket.timeout: break
                i = b.find(b'{"state"')
                if i >= 0:
                    d = 0
                    for j in range(i, len(b)):
                        d += (b[j:j+1] == b"{") - (b[j:j+1] == b"}")
                        if d == 0: js = json.loads(b[i:j+1]); break
            c.close()
            if js:
                ax = {a["segment"]: a for a in js.get("axes", []) if a.get("segment", "").startswith("drive_")}
                log("mib", state=js.get("state"), estop=js.get("estop", {}).get("latched"),
                    **{f"{k}_{f}": ax.get(f"drive_{k}", {}).get(f) for k in ("left", "right") for f in ("online", "age_ms", "state", "fault", "uptime_ms")})
                for k in ("left", "right"):
                    if ax.get(f"drive_{k}", {}).get("fault") not in (None, "NONE") and faulted["why"] is None:
                        faulted["why"] = f"drive_{k} fault {ax[f'drive_{k}']['fault']}"
        except Exception as e:
            log("mib_err", err=str(e)[:80])
        stop.wait(1.0)
def wheel_poll(name, ip):
    con = None
    while not stop.is_set():
        try:
            if con is None: con = el.EthConsole(ip)
            a = con.cmd("atos", wait=0.35); s = con.cmd("s", wait=0.35)
            g = lambda pat, txt, cast=int: (lambda m: cast(m.group(1)) if m else None)(re.search(pat, txt))
            log("wheel", wheel=name, rx=g(r"rx=(\d+)", a), sup_state=g(r"\| state=(\d+)", a), fault=g(r"fault=(\d+)", a),
                cmd_age=g(r"cmd_age=(\d+)ms", a), heap=g(r"heap free=(\d+)", a), heap_min=g(r"min=(\d+)", a),
                late_pct=g(r"\(([\d.]+)%\)", s, float), cmax_us=g(r"cmax=(\d+)us", s), isr_max=g(r"isr=[\d.]+/[\d.]+/([\d.]+)us", s, float))
        except Exception as e:
            log("wheel_err", wheel=name, err=str(e)[:80]); con = None
        stop.wait(1.5)
threads = [threading.Thread(target=mib_poll, daemon=True)] + [threading.Thread(target=wheel_poll, args=kv, daemon=True) for kv in WHEELS.items()]
for th in threads: th.start()

crowd = []
def add_crowd():
    for i in range(N):
        c = Context(); rclpy.init(context=c); crowd.append((c, rclpy.create_node("claude_crowd_%d" % i, context=c)))
    log("event", what=f"{N} participants up")
def del_crowd():
    for c, nd in crowd: nd.destroy_node(); rclpy.shutdown(context=c)
    log("event", what="participants removed")

phase = "idle"; armed = False; next_tw = 0; bucket = int(time.time() - t0); aborted = None
log("event", what="start", n=N)
try:
    while True:
        t = time.time() - t0
        ex.spin_once(timeout_sec=0.01)
        if int(t) != bucket:
            log("odom", hz=odom[0], state=st["state"]); odom[0] = 0; bucket = int(t)
        if phase == "idle" and t >= T_ARM:
            phase = "arming"; log("event", what="request AUTONOMOUS")
        if phase == "arming":
            if st["state"] == 4: phase = "armed"; armed = True; log("event", what="AUTONOMOUS")
            elif t - T_ARM > 3: aborted = "MIB did not enter AUTONOMOUS"; break
            elif int(t * 5) % 1 == 0: cmd(DriveCommand.REQUEST_AUTONOMOUS)
        if armed and time.time() >= next_tw:
            tw.publish(TwistSetpoint()); next_tw = time.time() + 0.05
        if phase == "armed":
            if st["state"] != 4: aborted = f"left AUTONOMOUS (state {st['state']})"; break
            if faulted["why"]: aborted = faulted["why"]; break
            if t >= T_ADD and not crowd and not getattr(add_crowd, "started", False):
                add_crowd.started = True; log("event", what=f"adding {N} participants"); threading.Thread(target=add_crowd, daemon=True).start()
            if t >= T_DEL and not getattr(del_crowd, "started", False):
                del_crowd.started = True; log("event", what="removing participants"); threading.Thread(target=del_crowd, daemon=True).start()
            if t >= T_DIS: phase = "disabling"; break
finally:
    if aborted: log("event", what="ABORT", why=aborted)
    for _ in range(10): tw.publish(TwistSetpoint()); ex.spin_once(timeout_sec=0.05)
    e = time.time() + 4
    while st["state"] not in (1, None) and time.time() < e: cmd(DriveCommand.REQUEST_DISABLE); ex.spin_once(timeout_sec=0.2)
    log("event", what="DISABLE", state=st["state"])
    if crowd and not getattr(del_crowd, "started", False): del_crowd()
    e = time.time() + 10
    while time.time() < e:
        ex.spin_once(timeout_sec=0.05); t = time.time() - t0
        if int(t) != bucket: log("odom", hz=odom[0], state=st["state"]); odom[0] = 0; bucket = int(t)
    stop.set(); log("event", what="end"); print("aborted:", aborted, "final state:", st["state"])
