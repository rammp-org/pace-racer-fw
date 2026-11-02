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
//  - FAST: a measured phase's raw conversion at the ADC's limits (railed) on N
//    consecutive conversions of that phase. The filter holds readings past the
//    plausibility gate, so without this a real short would never reach the
//    sustained check. D43: railed, not just past the gate, because readings at
//    38-40 A (gate 38, rail ~40) came in a run of 4 at ~33 A phase current on the chair.

namespace foc {

struct TripRule {
  int samples{3};           ///< sustained: over-limit samples (net, if leaky) to trip
  bool leaky{false};        ///< false = consecutive, true = +1 over / -1 clean
  int fast_conversions{0};  ///< fast: consecutive railed conversions; 0 = off
};

/// The sustained count after one sample.
inline int trip_count_step(int count, bool over, bool leaky) {
  if (over)
    return count + 1;
  if (!leaky)
    return 0;
  return count > 0 ? count - 1 : 0;
}

/// A phase's run of consecutive railed conversions after one conversion.
inline int run_step(int run, bool hit) { return hit ? run + 1 : 0; }

} // namespace foc
