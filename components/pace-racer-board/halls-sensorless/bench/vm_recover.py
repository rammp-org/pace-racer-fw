import serial, time, subprocess, sys
PSU = "/dev/cu.usbmodem615J231251"
import glob
def _port():
    p = [x for x in glob.glob("/dev/cu.usbmodem*") if "615J" not in x]
    return p[0] if p else "/dev/cu.usbmodem2101"
BOARD = _port()
IDFPY = "/Users/alxchunlin/.espressif/python_env/idf6.0_py3.11_env/bin/python"

def attempt():
    ps = serial.Serial(PSU, 115200, timeout=1.0)
    def q(c):
        ps.reset_input_buffer(); ps.write((c+"\n").encode()); time.sleep(0.25)
        return ps.read(200).decode(errors="replace").strip()
    ps.write(b"OUTP OFF\n")
    t0 = time.time()
    v = 99.0
    while time.time() - t0 < 20:
        time.sleep(1.0)
        try: v = float(q("MEAS:VOLT?") or 99)
        except ValueError: pass
        if v < 5.0: break
    ps.write(b"OUTP ON\n"); time.sleep(2.5)
    vm = q("MEAS:VOLT?")
    ps.close()
    if not vm or float(vm) < 40: return False, "VM did not return"
    r = subprocess.run([IDFPY, "-m", "esptool", "--chip", "esp32s3", "--port", BOARD,
                        "--before", "default-reset", "--after", "hard-reset", "read-mac"],
                       capture_output=True, text=True, timeout=40)
    if r.returncode != 0: return False, "esptool failed"
    time.sleep(3.0)
    s = serial.Serial(BOARD, 115200, timeout=0.4)
    time.sleep(0.5); s.reset_input_buffer()
    def cmd(c, wait=0.6):
        s.write((c+"\n").encode()); time.sleep(wait)
        return s.read(16000).decode(errors="replace")
    ok = True
    for lim, tok in (("lim 25 30", "#lim"), ("vds 4", "#vds"), ("vl 20", "#vl")):
        got = False
        for _ in range(4):
            if tok in cmd(lim): got = True; break
        ok = ok and got
    reg = cmd("r", 1.5)
    s.close()
    if "nfault_active=0" not in reg: return False, "nfault still active: " + reg[:200]
    return ok, reg[:200]

for i in range(3):
    ok, msg = attempt()
    print(f"attempt {i+1}: {'OK' if ok else 'FAIL'} — {msg.strip()[:150]}", flush=True)
    if ok: sys.exit(0)
sys.exit(1)
