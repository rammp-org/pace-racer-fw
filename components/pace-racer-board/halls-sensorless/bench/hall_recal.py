import sys, time
sys.path.insert(0, "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/components/pace-racer-board/halls-sensorless/bench")
from bench_lib import Board

b = Board()
try:
    print(b.cmd("hcal 6", wait=0.8), end="")
    for direction in (30, -30):
        print(b.cmd(f"run 8 {direction} 8", wait=0.8), end="")
        t0 = time.time()
        while time.time() - t0 < 12:
            time.sleep(1.5)
            hs = b.cmd("hs", wait=0.6)
            line = hs.splitlines()[0] if hs else ""
            print("  ", line)
            if "calleft=0/0" in line or ("calleft" in line and direction == 30 and line.split("calleft=")[1].startswith("0/")):
                break
        print(b.cmd("stop", wait=0.5), end="")
        time.sleep(1.0)
    print(b.cmd("hs", wait=0.8), end="")
finally:
    b.stop(); b.close()
