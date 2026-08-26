"""Self-check for tune.py analysis math against a synthetic RL plant."""

import math

from tune import fit_rl, metrics, steady_state

R, L, V = 0.8, 400e-6, 2.0  # tau = 0.5 ms
DT = 200e-6


def synth_open_loop(duration=0.15):
    tau = L / R
    n = int(duration / DT)
    return [(k * DT, (V / R) * (1 - math.exp(-k * DT / tau)), V) for k in range(1, n + 1)]


def test_fit_rl():
    r, l, tau, dt = fit_rl(synth_open_loop(), V)
    assert abs(r - R) / R < 0.02, r
    assert abs(l - L) / L < 0.15, l  # 63% crossing on coarse samples
    assert abs(dt - DT) / DT < 0.01, dt


def test_metrics_overdamped():
    rows = synth_open_loop()
    m = metrics(rows, V / R)
    assert m["overshoot_pct"] < 0.5, m
    assert abs(m["i_ss"] - V / R) / (V / R) < 0.02, m
    assert m["rise_ms"] > 0 and m["settle_ms"] > 0, m


def test_metrics_overshoot():
    # 20% overshoot hump slow enough to survive the median-7 filter (~1.4 ms window)
    rows = [(k * DT, 1.0 + 0.2 * math.exp(-k * DT / 8e-3) * math.cos(k * DT * 500), 1.0)
            for k in range(1, 500)]
    m = metrics(rows, 1.0)
    assert 10 < m["overshoot_pct"] < 25, m


def test_metrics_rejects_spikes():
    # flat 1.0 A with a lone 8 A ADC glitch: must NOT read as overshoot
    rows = [(k * DT, 8.0 if k == 100 else 1.0, 1.0) for k in range(1, 500)]
    m = metrics(rows, 1.0)
    assert m["overshoot_pct"] < 1.0, m


if __name__ == "__main__":
    test_fit_rl()
    test_metrics_overdamped()
    test_metrics_overshoot()
    test_metrics_rejects_spikes()
    print("ok")
