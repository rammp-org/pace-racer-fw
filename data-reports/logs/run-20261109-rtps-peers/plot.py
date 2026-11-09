import sys, json
sys.path.insert(0, "/Users/alxchunlin/code-stuff/alex-style-guide/matplotlib")
import alex_style as ax
import matplotlib.pyplot as plt
D = sys.argv[1]
p = ax.use("light")
r = [json.loads(l) for l in open(f"{D}/armed30.jsonl")]
ev = {x["what"]: x["t"] for x in r if x["kind"] == "event"}
t_arm, t_up, t_rm, t_dis = ev["AUTONOMOUS"], ev["adding 30 participants"], ev["removing participants"], ev["DISABLE"]
od = [x for x in r if x["kind"] == "odom"]
wh = [x for x in r if x["kind"] == "wheel"]
s = p["series"]
fig, axs = plt.subplots(4, 1, figsize=(9.6, 11), sharex=True)
def marks(a):
    for t, lab in ((t_arm, "AUTONOMOUS"), (t_up, "+30 participants"), (t_rm, "removed"), (t_dis, "DISABLE")):
        a.axvline(t, color=p["muted"], linestyle=(0, (2, 4)), linewidth=1)
        a.annotate(lab, xy=(t, 1), xycoords=("data", "axes fraction"), xytext=(3, -10), textcoords="offset points", color=p["muted"], fontsize=7.5)
a = axs[0]
a.plot([x["t"] for x in od], [x["hz"] for x in od], color=s[0], label="/base/odom")
a.set_ylabel("Odometry rate, msg/s"); a.set_ylim(0, 25); marks(a)
ax.title(a, "Odometry during the armed hold", "Messages per 1 s window at a Jetson subscriber. 0 at the start is ROS discovery.")
for i, (k, col) in enumerate((("left", s[0]), ("right", s[1]))):
    ww = [x for x in wh if x["wheel"] == k and x["cmd_age"] is not None]
    axs[1].plot([x["t"] for x in ww], [x["cmd_age"] for x in ww], color=col, label=f"drive_{k}")
    axs[2].plot([x["t"] for x in ww if x["late_pct"] is not None], [x["late_pct"] for x in ww if x["late_pct"] is not None], color=col, label=f"drive_{k}")
    axs[3].plot([x["t"] for x in ww if x["heap"]], [x["heap"] / 1024 for x in ww if x["heap"]], color=col, label=f"drive_{k}")
a = axs[1]; a.set_ylabel("Command age, ms"); a.set_ylim(0, 230); a.axhline(200, color=p["critical"], linestyle=(0, (4, 3)), linewidth=1); a.annotate("wheel command watchdog 200 ms", xy=(0.37, 200), xycoords=("axes fraction", "data"), xytext=(4, 4), textcoords="offset points", color=p["muted"], fontsize=8); marks(a); ax.label_lines(a, min_gap=22)
ax.title(a, "Age of the last MIB command at each wheel", "Sampled about every 1.5 s over the service port. Blue drive_left, red drive_right.")
a = axs[2]; a.set_ylabel("Sampler late, % of samples"); a.set_ylim(0, 3); marks(a); ax.label_lines(a, min_gap=0.25)
ax.title(a, "FOC current sampler lateness", "Late samples since the previous read ('s' console command).")
a = axs[3]; a.set_ylabel("Free internal heap, KB"); a.set_ylim(0, 70); a.axhline(10.0, color=p["critical"], linestyle=(0, (4, 3)), linewidth=1); a.annotate("0.4.8 with raised limits: minimum 10 KB, then rebooted", xy=(0.37, 10.0), xycoords=("axes fraction", "data"), xytext=(4, 4), textcoords="offset points", color=p["muted"], fontsize=8); marks(a); ax.label_lines(a, min_gap=6)
ax.title(a, "Free internal heap", "'atos' console line, MALLOC_CAP_INTERNAL.")
[x.set_xlabel("Time since test start, s") for x in axs]; [x.tick_params(labelbottom=True) for x in axs]
fig.tight_layout(h_pad=2.2)
ax.save(fig, f"{D}/armed_hold_30.png", dpi=150)

import csv
rows = list(csv.DictReader(open(f"{D}/ram.csv")))
fig, a = plt.subplots(figsize=(9, 2.8))
labels = ["0.4.7", "0.4.8, limits raised to 24", "0.4.8, espp-only peers"]
vals = [round(int(x["domain_bytes"]) / 1024, 1) for x in rows]
ax.bars(a, labels, vals, highlight="0.4.8, limits raised to 24", unit=" KB", xlabel="RTPS Domain object, KB", ylabel="Wheel firmware")
ax.title(a, "RAM taken by the RTPS domain", "sizeof(rtps::Domain) from each ELF. Red: the build that ran out of memory and was rolled back.")
fig.tight_layout(); ax.save(fig, f"{D}/rtps_domain_ram.png", dpi=150)
print("ok")
