"""Smoke test: CSV parse() stays in lockstep with sensorless.cpp's 16 columns,
and the derived quantities behave."""
import math

from monitor import Row, parse

good = ("12.30, C, 0.05, 6.20, 6.25, 0.12, 4.80, 2, 150, 151, 149, 0.0261, "
        "41.5, 44.2, 39.8, 43.1")
r = parse(good)
assert r is not None
assert r.mode == "C" and r.iq == 6.20 and r.iqref == 6.25
assert r.temps == (41.5, 44.2, 39.8, 43.1)
assert abs(r.p_w - 1.5 * (0.12 * 0.05 + 4.80 * 6.20)) < 1e-6
assert abs(r.ibus - r.p_w / 48.0) < 1e-6
assert abs(r.vmag - math.hypot(0.12, 4.80)) < 1e-6

nan_temps = ("0.10, H, 0.00, 0.00, 0.00, 0.00, 0.00, 0, 0, 0, 0, 0.0000, "
             "nan, nan, nan, nan")
r = parse(nan_temps)
assert r is not None and all(math.isnan(t) for t in r.temps)

assert parse("%t, mode, id, iq, ...") is None       # header skipped
assert parse("#ready bus=48.0V ...") is None        # echo skipped
assert parse("! soft trip >15A on A ...") is None   # alert skipped
assert parse("") is None                            # blank skipped
assert parse("1,2,3") is None                       # wrong column count
assert parse("1, CC, " + "0, " * 13 + "0") is None  # bad mode column
print("ok")
