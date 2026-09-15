import serial, time, sys
s = serial.Serial("/dev/cu.usbmodem2101", 115200, timeout=0.4)
time.sleep(0.4)
last = "?"
t0 = time.time()
while time.time() - t0 < 600:
    s.reset_input_buffer()
    s.write(b"hs\n"); time.sleep(0.7)
    ok = "#hall" in s.read(16000).decode(errors="replace")
    now = "ALIVE" if ok else "MUTE"
    if now != last:
        print(f"[{time.time()-t0:5.0f}s] board {now}", flush=True)
        last = now
    time.sleep(2.0)
print("watch done — final:", last, flush=True)
s.close()
