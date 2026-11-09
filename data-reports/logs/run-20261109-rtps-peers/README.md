# RTPS crowd test on the flatbot, 2026-11-09

Wheel fw 0.4.8 (`1f677c2` code, branch `alexchunlin/racer-rtps-espp-peers`) against extra ROS 2 participants on the robot LAN (`eno1` on snorlax). MIB: `f204b8e-dirty`, the RAMMP-mib-fw#77 test build. Wheels: flatbot drive_left (`10:bd:a3:64:ee:fc`) and drive_right (`10:bd:a3:66:19:f0`). Nothing moved: idle tests are DISARMED, the armed test holds zero twist.

| File | What |
|---|---|
| `armed30.jsonl` | Armed zero-twist hold, 30 extra participants from t=30 s to t=90 s. One JSON object per sample: `odom` (msgs per 1 s window), `mib` (MIB status over tcp :5555, about 1 Hz), `wheel` (wheel `atos` and `s` lines, about every 1.5 s), `event`. |
| `armed30.py` | Runs that test from snorlax. |
| `idle_both_30.txt`, `stress2.sh` | Both wheels idle, 30 extra participants for 60 s. |
| `idle_right_30.txt`, `stress_right.sh` | Right wheel alone, same load (left wheel was hung on 0.4.7 at the time). |
| `many.py` | Creates N idle ROS 2 participants in one process, one rclpy context each. |
| `ram.csv` | `sizeof(rtps::Domain)` and `sizeof(rtps::Participant)` from each ELF, and heap on the board. |
| `plot.py` | Makes the two PNGs (needs `alex_style.py` from the style guide). |
