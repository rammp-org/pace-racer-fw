#pragma once
// Host stand-in for ESP-IDF's esp_timer: a clock the test advances by hand.
#include <cstdint>
inline int64_t g_fake_now_us = 0;
inline int64_t esp_timer_get_time() { return g_fake_now_us; }
