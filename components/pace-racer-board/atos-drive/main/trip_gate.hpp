#pragma once

// Software overcurrent trip decision, per motor profile (D41). Integer only:
// it runs in the sampler ISR, where the Xtensa FPU is off limits.
//
// Two ways to trip:
//  - SUSTAINED: the filtered current of any phase (A, B or the reconstructed
//    C) over the trip level. Each ISR sample over the limit adds 1, each clean
//    one either resets the count (consecutive, the original rule) or takes 1
//    off it (leaky: a net count, so noise that is over half the time or less
//    never trips, while a real overcurrent with a few clean samples still does).
//  - FAST: a measured phase's raw conversion past the plausibility gate
//    (trip + margin) on N consecutive conversions of that phase. The filter
//    holds such readings, so without this a real short above the gate would
//    never reach the sustained check; noise does not repeat N times in a row.

namespace foc {

struct TripRule {
  int samples{3};           ///< sustained: over-limit samples (net, if leaky) to trip
  bool leaky{false};        ///< false = consecutive, true = +1 over / -1 clean
  int fast_conversions{0};  ///< fast: consecutive implausible conversions; 0 = off
};

/// The sustained count after one sample.
inline int trip_count_step(int count, bool over, bool leaky) {
  if (over)
    return count + 1;
  if (!leaky)
    return 0;
  return count > 0 ? count - 1 : 0;
}

/// A phase's run of consecutive implausible conversions after one conversion.
inline int implausible_run_step(int run, bool implausible) { return implausible ? run + 1 : 0; }

} // namespace foc
