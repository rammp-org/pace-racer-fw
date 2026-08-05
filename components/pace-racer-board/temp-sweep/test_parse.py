"""Smoke test: CSV parse() stays in lockstep with temp_sweep.cpp's 11 columns."""
from monitor import parse

good = "1.234, 30.0, 29.87, 1, 0.101, -0.050, 0.033, 25.10, 26.00, 24.90, 27.30"
r = parse(good)
assert r is not None
assert r.target_rpm == 30.0 and r.enabled == 1.0
assert r.temps == (25.10, 26.00, 24.90, 27.30)

assert parse("%header line") is None          # header skipped
assert parse("") is None                       # blank skipped
assert parse("1,2,3") is None                  # wrong column count
assert parse("a,b,c,d,e,f,g,h,i,j,k") is None  # non-numeric
print("ok")
