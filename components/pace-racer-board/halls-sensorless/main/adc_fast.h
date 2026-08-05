#pragma once

// Minimal C shim over the ADC low-level layer.
//
// The sampling ISR needs to change only the ADC channel between conversions —
// everything else adc_oneshot_hal_setup() programs is invariant here, and
// hoisting it out is worth ~7 us per conversion. The call that does it,
// adc_oneshot_ll_set_channel(), lives in "hal/adc_ll.h", which does not compile
// as C++: it typedefs sens_dev_t, colliding with adc_oneshot_hal.h's
// forward declaration, and its designated initializers trip
// -Werror=missing-field-initializers. So it is confined to a C translation unit.

#ifdef __cplusplus
extern "C" {
#endif

/// Select the ADC channel for the next oneshot conversion on the given unit.
/// Both arguments are the plain integer values of adc_unit_t / adc_channel_t.
/// Safe to call from an ISR: it is a single register write.
void adc_fast_select_channel(int unit, int channel);

#ifdef __cplusplus
}
#endif
