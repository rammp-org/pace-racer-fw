#!/usr/bin/env python3
"""
PACE RACER phase-current PI auto-tuner.

Talks to the current-control firmware over the USB console:
  1. Open-loop voltage step  -> fit R, L of the A->B winding path.
  2. Compute PI gains analytically (pole-zero cancellation):
       kp = L*wc, ki = R*wc  ->  first-order closed loop, no overshoot
       by construction; wc chosen from the requested settle time.
  3. Closed-loop verification steps; nudge wc down if discrete-time effects
     cause overshoot, up if the response is needlessly slow. Stops when the
     response is slightly overdamped: <=2% overshoot, settle near target.

Every dump is saved as CSV (and PNG with --plot) in --out.

Usage:
    python tune.py [--port /dev/cu.usbmodem2101] [--target 3.0] [--settle-ms 10]
"""

import argparse
import math
import time
from pathlib import Path

import serial

# ---------------------------------------------------------------- serial I/O


def read_dump(ser, timeout_s=10.0):
    """Collect one #dump ... #end frame. Returns (header, rows, status).

    status is "ok", "trip" (software overcurrent, retryable), or
    "fault 0x...." (DRV8353 hardware OCP latched — stop and investigate).
    """
    header, rows, in_dump = None, [], False
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        if line.startswith("#dump"):
            header, rows, in_dump = line, [], True
        elif line.startswith("#end"):
            if in_dump:
                status = line[len("#end "):]
                if status.startswith("fault"):
                    raise SystemExit(
                        f"DRV8353 hardware OCP latched ({status}) — driver is shut "
                        "down and stays latched. Inspect the board/wiring, then "
                        "power-cycle before tuning again.")
                return header, rows, status
        elif in_dump:
            parts = line.split(",")
            if len(parts) == 3:
                try:
                    rows.append((int(parts[0]) * 1e-6, float(parts[1]), float(parts[2])))
                except ValueError:
                    pass  # log noise interleaved with the dump
    raise TimeoutError("no #end from firmware — is it flashed and connected?")


def command(ser, cmd, timeout_s=10.0):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    return read_dump(ser, timeout_s)


# ---------------------------------------------------------------- analysis


def steady_state(rows):
    tail = rows[int(len(rows) * 0.8):]
    return sum(r[1] for r in tail) / len(tail)


def fit_rl(rows, volts):
    """Fit I=(V/R)(1-e^-t/tau) via steady state + 63.2% crossing."""
    i_ss = steady_state(rows)
    r = volts / i_ss
    thresh = 0.632 * i_ss
    tau = None
    for (t0, i0, _), (t1, i1, _) in zip(rows, rows[1:]):
        if (i0 - thresh) * (i1 - thresh) <= 0 and i0 != i1:
            tau = t0 + (thresh - i0) / (i1 - i0) * (t1 - t0)
            break
    dt = rows[-1][0] / len(rows)
    if tau is None or tau < 2 * dt:
        tau = 0.0  # plant faster than the loop can resolve; integrator-only design
    return r, r * tau, tau, dt


def median_filter(vals, half=3):
    """Rolling median-of-7: rejects single-sample ADC spikes without lag bias."""
    out = []
    for k in range(len(vals)):
        w = sorted(vals[max(0, k - half):k + half + 1])
        out.append(w[len(w) // 2])
    return out


def metrics(rows, target):
    """Step metrics on median-filtered data with a noise-aware settle band.

    The raw stream has ~0.2 A quantization (1 ADC LSB at 5 mV/A) plus spikes;
    raw peaks and a 2%-of-target settle band would report garbage.
    """
    import statistics

    i_raw = [r[1] for r in rows]
    t = [r[0] for r in rows]
    i = median_filter(i_raw)
    tail = i_raw[int(len(i_raw) * 0.8):]
    i_ss = sum(tail) / len(tail)
    noise = statistics.pstdev(tail)
    peak = max(i)
    overshoot = max(0.0, (peak - i_ss) / abs(i_ss)) * 100 if i_ss else float("inf")
    t10 = t90 = settle = None
    for tk, ik in zip(t, i):
        if t10 is None and ik >= 0.1 * i_ss:
            t10 = tk
        if t90 is None and ik >= 0.9 * i_ss:
            t90 = tk
    band = max(0.02 * abs(i_ss), 1.5 * noise)
    for tk, ik in zip(reversed(t), reversed(i)):
        if abs(ik - i_ss) > band:
            settle = tk
            break
    return {
        "i_ss": i_ss,
        "noise_a": noise,
        "overshoot_pct": overshoot,
        "rise_ms": (t90 - t10) * 1e3 if t10 is not None and t90 is not None else float("nan"),
        "settle_ms": (settle or 0.0) * 1e3,
        "err_pct": (i_ss - target) / target * 100 if target else float("nan"),
    }


# ---------------------------------------------------------------- output


def save_csv(rows, path):
    with open(path, "w") as f:
        f.write("t_s,i_a,u_v\n")
        f.writelines(f"{t:.6f},{i:.4f},{u:.4f}\n" for t, i, u in rows)


def save_plot(rows, target, title, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    t = [r[0] * 1e3 for r in rows]
    surface, ink, ink2 = "#fcfcfb", "#0b0b0b", "#52514e"
    fig, (ax_i, ax_u) = plt.subplots(
        2, 1, sharex=True, figsize=(8, 5),
        height_ratios=[3, 1], facecolor=surface)
    for ax in (ax_i, ax_u):
        ax.set_facecolor(surface)
        ax.grid(True, color="#e5e4e0", linewidth=0.7)
        ax.tick_params(colors=ink2, labelsize=9)
        for s in ax.spines.values():
            s.set_visible(False)
    if target is not None:
        ax_i.axhline(target, color=ink2, linestyle="--", linewidth=1)
        ax_i.annotate(f"target {target:g} A", (t[-1], target), color=ink2,
                      fontsize=9, ha="right", va="bottom")
    ax_i.plot(t, [r[1] for r in rows], color="#2a78d6", linewidth=2)
    ax_i.set_ylabel("current (A)", color=ink2, fontsize=9)
    ax_i.set_title(title, color=ink, fontsize=11, loc="left")
    ax_u.plot(t, [r[2] for r in rows], color="#1baf7a", linewidth=2)
    ax_u.set_ylabel("drive (V)", color=ink2, fontsize=9)
    ax_u.set_xlabel("time (ms)", color=ink2, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------- main


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="/dev/cu.usbmodem2101")
    p.add_argument("--target", type=float, default=3.0, help="verification step amps")
    p.add_argument("--settle-ms", type=float, default=10.0, help="desired 2%% settle time")
    p.add_argument("--max-iters", type=int, default=8)
    p.add_argument("--out", default="tune_logs")
    p.add_argument("--plot", action="store_true", help="save a PNG per step")
    args = p.parse_args()

    out = Path(args.out)
    out.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")

    ser = serial.Serial(args.port, 115200, timeout=1.0)
    time.sleep(0.5)
    ser.reset_input_buffer()

    def record(rows, target, name):
        save_csv(rows, out / f"{stamp}-{name}.csv")
        if args.plot:
            save_plot(rows, target, name, out / f"{stamp}-{name}.png")

    # ---- 1. identification: voltage step, escalate until measurable current.
    # Start low: winding resistance may be tens of mΩ, where even small voltages
    # drive amps. A trip halves the voltage; a too-small response doubles it.
    volts, rows = 0.1, []
    for attempt in range(8):
        header, rows, status = command(ser, f"v {volts:.3f} 150")
        if status != "ok":
            record(rows, None, f"id-{volts:.2f}V-{status}")
            peak = max((abs(r[1]), r[0]) for r in rows) if rows else (0, 0)
            print(f"ID @ {volts:.2f} V tripped (peak |i|={peak[0]:.2f} A at "
                  f"t={peak[1] * 1e3:.2f} ms) — halving voltage")
            volts /= 2
            if volts < 0.02:
                raise SystemExit("tripping even at 20 mV — something is wrong, see dumps")
            continue
        if not rows:
            raise SystemExit("empty dump — check firmware")
        i_ss = steady_state(rows)
        if i_ss < 0 and abs(i_ss) > 0.2:
            print(f"ID: negative response ({i_ss:.2f} A) — flipping feedback sign")
            ser.write(b"s -1\n")
            time.sleep(0.2)
            continue
        if i_ss > 0.3:
            break
        volts *= 2
        if volts > 4.0:
            raise SystemExit(f"no current response up to 4 V (i_ss={i_ss:.3f} A) — wiring?")
    else:
        raise SystemExit("could not find a workable ID voltage — see dumps")
    record(rows, None, f"id-{volts:.2f}V")

    r, l, tau, dt = fit_rl(rows, volts)
    print(f"ID @ {volts:.2f} V: i_ss={steady_state(rows):.2f} A  "
          f"R={r:.3f} ohm  L={l * 1e6:.0f} uH  tau={tau * 1e3:.2f} ms  "
          f"(per phase: R={r / 2:.3f}, L={l / 2 * 1e6:.0f} uH)  loop dt={dt * 1e6:.0f} us")
    if tau == 0.0:
        print("   plant faster than loop rate — integrator-dominant design (kp from dt)")

    # ---- 2/3. compute gains, verify, iterate
    wc_cap = 2 * math.pi / dt / 20  # crossover well below the sample rate
    wc = min(4.0 / (args.settle_ms * 1e-3), wc_cap)
    step_ms = int(max(6 * args.settle_ms, 100))

    # Throwaway step: torque the rotor to its detent so verification steps
    # measure the RL+PI loop, not the rotor-motion BEMF transient.
    kp, ki = l * wc, r * wc
    command(ser, f"c {kp:.6g} {ki:.6g} {args.target:.3f} {step_ms}")

    for it in range(1, args.max_iters + 1):
        kp, ki = l * wc, r * wc
        header, rows, status = command(ser, f"c {kp:.6g} {ki:.6g} {args.target:.3f} {step_ms}")
        if status != "ok":
            print(f"iter {it}: overcurrent trip — halving bandwidth")
            wc *= 0.5
            continue
        m = metrics(rows, args.target)
        record(rows, args.target, f"step{it}")
        rise_expect_ms = 2.2 / wc * 1e3  # 10-90 rise of a first-order loop at wc
        print(f"iter {it}: kp={kp:.4g} ki={ki:.4g} wc={wc:.0f} rad/s | "
              f"i_ss={m['i_ss']:.2f} A ({m['err_pct']:+.1f}%) "
              f"overshoot={m['overshoot_pct']:.1f}% rise={m['rise_ms']:.1f} ms "
              f"(expect {rise_expect_ms:.1f}) settle={m['settle_ms']:.1f} ms "
              f"noise={m['noise_a']:.2f} A")

        if abs(m["err_pct"]) > 5.0:
            print(f"iter {it}: steady-state error {m['err_pct']:+.1f}% — check setup")
        if m["overshoot_pct"] > 5.0:
            wc *= 0.75
        elif m["rise_ms"] > 3.0 * rise_expect_ms and wc < wc_cap:
            wc = min(wc * 1.2, wc_cap)
        else:
            print(f"\nDONE — slightly overdamped response achieved.")
            print(f"  A->B path:  kp = {kp:.6g} V/A,  ki = {ki:.6g} V/(A*s)")
            print(f"  plant:      R = {r:.4f} ohm, L = {l * 1e6:.1f} uH "
                  f"(per phase: {r / 2:.4f} ohm, {l / 2 * 1e6:.1f} uH)")
            print(f"  logs in {out}/")
            return

    print(f"\nno convergence in {args.max_iters} iterations — see logs in {out}/")


if __name__ == "__main__":
    main()
