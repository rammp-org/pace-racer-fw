// Host test for the D41 soft-trip rule (trip_gate.hpp) with each profile's
// constants: what a burst of noise, a real overcurrent and a short do to it.
// One sample = one sampler ISR tick (50 us at 20 kHz); phases alternate, so
// each measured phase converts every other tick.

#include <cstdio>
#include <vector>

#include "motor_params.hpp"
#include "trip_gate.hpp"

static int g_fail = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
      g_fail++;                                                                                    \
    }                                                                                              \
  } while (0)

static const foc::TripRule kRule{motor::kTripSamples, motor::kTripLeaky,
                                 motor::kTripFastConversions};

/// Feed an over/clean pattern to the sustained check; the tick it trips at, or -1.
static int sustained_trip_tick(const std::vector<bool> &over) {
  int count = 0;
  for (size_t i = 0; i < over.size(); i++) {
    count = foc::trip_count_step(count, over[i], kRule.leaky);
    if (count >= kRule.samples)
      return (int)i;
  }
  return -1;
}

static std::vector<bool> pattern(int n, auto over_at) {
  std::vector<bool> v(n);
  for (int i = 0; i < n; i++)
    v[i] = over_at(i);
  return v;
}

static void test_sustained() {
  // A real overcurrent: over the limit from tick 0. Trips after kTripSamples.
  CHECK(sustained_trip_tick(pattern(200, [](int) { return true; })) == kRule.samples - 1);

  // The chair's trip (2026-11-01): a 3-sample burst over the limit, then clean.
  // The old rule (3 consecutive) fires on it; hub20's must not.
  const int burst = sustained_trip_tick(pattern(200, [](int i) { return i < 3; }));
  if (kRule.leaky)
    CHECK(burst == -1);
  else
    CHECK(burst == 2);

  // Alternating over/clean (one phase misreading every other conversion) for
  // 50 ms: the leaky count never builds; the consecutive rule never sees 2 in a row.
  CHECK(sustained_trip_tick(pattern(1000, [](int i) { return i % 2 == 0; })) == -1);

  if (kRule.leaky) {
    // Repeated 3-sample bursts with 2 clean samples between: net +1 per 5 ticks,
    // so a persistent mostly-over condition still trips, after net kTripSamples.
    const int t = sustained_trip_tick(pattern(2000, [](int i) { return i % 5 < 3; }));
    CHECK(t > 0);
    std::printf("  %s: 3-over/2-clean trips at %.2f ms\n", motor::kName, (t + 1) * 0.05f);
    // A real overcurrent with an occasional clean (misread) sample still trips
    // within about 1.1x the clean time.
    const int t2 = sustained_trip_tick(pattern(200, [](int i) { return i % 10 != 9; }));
    CHECK(t2 >= kRule.samples - 1 && t2 <= kRule.samples * 13 / 10);
    std::printf("  %s: sustained trip after %.2f ms (%.2f ms with 1-in-10 clean)\n", motor::kName,
                kRule.samples * 0.05f, (t2 + 1) * 0.05f);
  }
}

static void test_fast() {
  if (kRule.fast_conversions <= 0) {
    CHECK(motor::kTripFastConversions == 0);
    return;
  }
  // A short: the phase reads railed on every conversion.
  int run = 0, at = -1;
  for (int c = 0; c < 20 && at < 0; c++) {
    run = foc::run_step(run, true);
    if (run >= kRule.fast_conversions)
      at = c;
  }
  CHECK(at == kRule.fast_conversions - 1);
  std::printf("  %s: fast trip after %d conversions (%.2f ms at alternating phases)\n",
              motor::kName, at + 1, (at + 1) * 2 * 0.05f);
  // The chair's false fast trip (D43): 4 railed-looking conversions in a row,
  // then sane ones. Must not trip.
  run = 0;
  for (int c = 0; c < 4; c++)
    run = foc::run_step(run, true);
  run = foc::run_step(run, false);
  CHECK(run == 0 && kRule.fast_conversions > 4);
  // A burst one shorter than the run, then a sane reading, repeated: no trip.
  run = 0;
  bool tripped = false;
  for (int c = 0; c < 100; c++) {
    run = foc::run_step(run, c % kRule.fast_conversions != 0);
    tripped |= run >= kRule.fast_conversions;
  }
  CHECK(!tripped);
}

int main() {
  std::printf("profile %s: trip %d %s samples, fast path %d conversions\n", motor::kName,
              kRule.samples, kRule.leaky ? "net" : "consecutive", kRule.fast_conversions);
  test_sustained();
  test_fast();
  std::printf("%s\n", g_fail ? "FAILED" : "ok");
  return g_fail ? 1 : 0;
}
