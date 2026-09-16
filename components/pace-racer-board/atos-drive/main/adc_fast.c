#include "adc_fast.h"

#include "esp_attr.h"
#include "hal/adc_ll.h"

IRAM_ATTR void adc_fast_select_channel(int unit, int channel) {
  adc_oneshot_ll_set_channel((adc_unit_t)unit, (adc_channel_t)channel);
}
