#include <numbers>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Tune this until one mechanical revolution shows exactly 6*PP transitions
static constexpr int kPolePairs = 15;

static constexpr gpio_num_t kHallA = GPIO_NUM_3;
static constexpr gpio_num_t kHallB = GPIO_NUM_46;
static constexpr gpio_num_t kHallC = GPIO_NUM_9;

// hall state (3-bit index) → sector 0-5, or -1 if invalid
// standard 120°-apart forward sequence: 101→100→110→010→011→001
// reorder if your motor spins backwards
static constexpr int kHallToSector[8] = {-1, 5, 3, 4, 1, 0, 2, -1};

struct HallEvent {
  uint8_t  state;
  int64_t  timestamp_us;
};

static QueueHandle_t hall_queue;

static void IRAM_ATTR hall_isr(void *) {
  HallEvent ev{
    .state        = (uint8_t)((gpio_get_level(kHallA) << 2) |
                              (gpio_get_level(kHallB) << 1) |
                               gpio_get_level(kHallC)),
    .timestamp_us = esp_timer_get_time(),
  };
  xQueueSendFromISR(hall_queue, &ev, nullptr);
}

extern "C" void app_main(void) {
  gpio_config_t cfg{
    .pin_bit_mask = (1ULL << kHallA) | (1ULL << kHallB) | (1ULL << kHallC),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_ANYEDGE,
  };
  gpio_config(&cfg);

  hall_queue = xQueueCreate(32, sizeof(HallEvent));

  gpio_install_isr_service(0);
  gpio_isr_handler_add(kHallA, hall_isr, nullptr);
  gpio_isr_handler_add(kHallB, hall_isr, nullptr);
  gpio_isr_handler_add(kHallC, hall_isr, nullptr);

  printf("\nHall calibration — PP=%d, transitions_per_rev=%d\n\n", kPolePairs, kPolePairs * 6);

  printf("Sector map (standard, verify by spinning CW and checking angle increases):\n");
  printf("  state | A B C | sector | elec_deg\n");
  for (int st = 1; st <= 6; st++) {
    int sec = kHallToSector[st];
    printf("  %d     | %d %d %d | %d      | %.0f\n",
           st, (st >> 2) & 1, (st >> 1) & 1, st & 1,
           sec, sec * 60.0f);
  }

  printf("\ntimestamp_us, A, B, C, state, sector, elec_deg, mech_deg, transitions\n");

  int  transitions   = 0;
  int  prev_sector   = -1;
  float cumulative_elec_deg = 0.0f;

  HallEvent ev;
  while (true) {
    xQueueReceive(hall_queue, &ev, portMAX_DELAY);

    int state  = ev.state & 0x7;
    int sector = kHallToSector[state];
    int ha     = (state >> 2) & 1;
    int hb     = (state >> 1) & 1;
    int hc     =  state & 1;

    if (sector < 0) {
      printf("%lld, %d, %d, %d, %d, INVALID, -, -, %d\n",
             ev.timestamp_us, ha, hb, hc, state, transitions);
      continue;
    }

    // accumulate angle from sector-to-sector delta to track direction
    if (prev_sector >= 0) {
      int delta = sector - prev_sector;
      if (delta >  3) delta -= 6;
      if (delta < -3) delta += 6;
      cumulative_elec_deg += delta * 60.0f;
    }
    prev_sector = sector;
    transitions++;

    float mech_deg = cumulative_elec_deg / kPolePairs;

    printf("%lld, %d, %d, %d, %d, %d, %.0f, %.2f, %d\n",
           ev.timestamp_us, ha, hb, hc, state, sector,
           sector * 60.0f, mech_deg, transitions);
  }
}
