// Host test for the D43 rest hold (rest_hold.hpp): when the anchor is taken,
// and the pull-back setpoint around it. Constants as in atos_drive.cpp.

#include <cstdio>

#include "rest_hold.hpp"

static int g_fail = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
      g_fail++;                                                                                    \
    }                                                                                              \
  } while (0)

static constexpr float kStoppedRpm = 2.0f, kSettleS = 0.25f, kPerStep = 1.0f, kMaxRpm = 10.0f;

int main() {
  using foc::rest_pullback_rpm;
  using foc::rest_settled;
  // Anchoring. The chair's case: the speed reads ~0 at the overshoot's zero
  // crossing but a hall step came 50 ms ago, so no anchor yet.
  CHECK(!rest_settled(0.5f, 0.05f, kStoppedRpm, kSettleS));
  CHECK(!rest_settled(-3.0f, 1.0f, kStoppedRpm, kSettleS)); // still moving
  CHECK(rest_settled(-0.5f, 0.30f, kStoppedRpm, kSettleS));
  CHECK(rest_settled(0.0f, kSettleS, kStoppedRpm, kSettleS));

  // Pull-back: nothing inside +-1 step, then 1 rpm per step beyond, clamped.
  CHECK(rest_pullback_rpm(0, kPerStep, kMaxRpm) == 0.0f);
  CHECK(rest_pullback_rpm(1, kPerStep, kMaxRpm) == 0.0f);
  CHECK(rest_pullback_rpm(-1, kPerStep, kMaxRpm) == 0.0f);
  CHECK(rest_pullback_rpm(2, kPerStep, kMaxRpm) == 1.0f);  // was a 10 rpm kick
  CHECK(rest_pullback_rpm(-2, kPerStep, kMaxRpm) == -1.0f);
  CHECK(rest_pullback_rpm(5, kPerStep, kMaxRpm) == 4.0f);
  CHECK(rest_pullback_rpm(50, kPerStep, kMaxRpm) == kMaxRpm); // a big slip still pulls back
  CHECK(rest_pullback_rpm(-50, kPerStep, kMaxRpm) == -kMaxRpm);
  // Monotonic in the error: no step in the setpoint at the deadband edge
  // bigger than one step's worth.
  for (int e = 1; e < 30; e++)
    CHECK(rest_pullback_rpm(e + 1, kPerStep, kMaxRpm) - rest_pullback_rpm(e, kPerStep, kMaxRpm) <=
          kPerStep);

  std::printf("%s\n", g_fail ? "FAILED" : "ok");
  return g_fail ? 1 : 0;
}
