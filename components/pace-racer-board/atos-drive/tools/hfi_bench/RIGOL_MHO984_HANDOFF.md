# Rigol MHO984 waveform capture — handoff

Verified working 2026-10-23 from this Mac.

## Connection

- Scope: `RIGOL TECHNOLOGIES,MHO984,MHO9B286M00520,00.01.00`
- IP `192.168.1.101`, routed via **en7** (bench switch). Ping ~1 ms.
- **Raw SCPI over TCP port 5555.** No VISA, no pyvisa, no drivers needed. (80 = web UI, 111 = VXI-11 portmapper; ignore both.)
- `:SYST:ERR?` returned `0,"No error"` after a full capture.

## Working script

Usage: `python3 grab.py [host] [CHANn]`. Writes `CHANn.csv` (`t_s,v`) next to where you run it.

```python
import socket, struct, sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.101"
CH = sys.argv[2] if len(sys.argv) > 2 else "CHAN1"

s = socket.create_connection((HOST, 5555), timeout=5)
f = s.makefile("rb")

def w(cmd): s.sendall((cmd + "\n").encode())
def q(cmd): w(cmd); return f.readline().decode().strip()

def read_block():
    # IEEE 488.2 definite-length block: #<n><n len digits><data>\n
    assert f.read(1) == b"#"
    n = int(f.read(1))
    length = int(f.read(n))
    data = f.read(length)   # read EXACTLY length bytes; never readline() binary
    f.readline()            # trailing \n
    return data

print("IDN:", q("*IDN?"))
w(":WAV:SOUR " + CH)
w(":WAV:MODE NORM")   # on-screen points (1000); RAW needs :STOP first, see below
w(":WAV:FORM WORD")   # 16-bit little-endian; MHO is 12-bit so BYTE loses resolution
pre = q(":WAV:PRE?").split(",")
# format,type,points,count,xinc,xorig,xref,yinc,yorig,yref
xinc, xorig, xref = map(float, pre[4:7])
yinc, yorig, yref = map(float, pre[7:10])
w(":WAV:DATA?")
raw = read_block()
codes = struct.unpack("<%dH" % (len(raw) // 2), raw)
volts = [(c - yorig - yref) * yinc for c in codes]
t = [xorig + (i - xref) * xinc for i in range(len(volts))]
print(f"{len(volts)} pts, dt={xinc:g}s, Vmin={min(volts):.4g} Vmax={max(volts):.4g}")
with open(f"{CH}.csv", "w") as out:
    out.write("t_s,v\n")
    out.writelines(f"{a:.9g},{b:.6g}\n" for a, b in zip(t, volts))
```

Actual output from the verified run:

```
IDN: RIGOL TECHNOLOGIES,MHO984,MHO9B286M00520,00.01.00
preamble: ['1', '0', '1000', '1', '5.000000E-4', '-5.000000E-1', '0.000000', '6.6667E-06', '0', '32768']
1000 pts, dt=0.0005s, Vmin=0.00146 Vmax=0.00502
```

(CH1 had nothing connected: ~3 mV noise at 50 mV/div.)

## Gotchas (likely where things went wrong before)

1. **Terminate every command with `\n`.** Query replies are one `\n`-terminated line.
2. **`:WAV:DATA?` returns a binary block, not text.** Parse the `#<n><len>` header and read exactly `len` bytes. `readline()` or `recv(4096)` on the payload will hang or truncate.
3. **Use `:WAV:FORM WORD`, not BYTE.** WORD preamble gives `yref=32768`; BYTE gives `yref=128`. Always take `yinc/yorig/yref` from `:WAV:PRE?` *after* setting the format, since they change with it.
4. **Voltage = `(code - yorig - yref) * yinc`.** Time = `xorig + (i - xref) * xinc`.
5. **NORM mode = only the 1000 on-screen points.** For full memory depth (was 1 Mpts at 2 MSa/s):
   `:STOP`, then `:WAV:MODE RAW`, then read in chunks with `:WAV:STAR <n>` / `:WAV:STOP <m>` / `:WAV:DATA?`. Not yet tested here; `:STOP` freezes the scope, so check nobody else is using it. `:RUN` afterwards.
6. Only CH1 was enabled at check time (`:CHAN2:DISP?` → 0). Enable a channel with `:CHANn:DISP 1` before sourcing it.

## Scope state at time of check

| Setting | Value |
|---|---|
| CH1 | on, 50 mV/div |
| CH2 | off |
| Timebase | 50 ms/div |
| Trigger | AUTO |
| Memory depth | 1 Mpts |
| Sample rate | 2 MSa/s |
