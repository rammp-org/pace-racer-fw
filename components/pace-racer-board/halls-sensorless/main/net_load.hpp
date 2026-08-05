#pragma once

// Network load generator + transmit benchmark for the ethernet/FOC coexistence
// work.
//
// The W5500 sits on SPI2 — the SAME bus as the DRV8353 — and raises an interrupt
// on GPIO14, so the original question was whether carrying traffic disturbs the
// 20 kHz control loop. Measured answer (2026-08-03): idle ethernet is free, and
// disturbance scales with PACKET rate, not bandwidth (3961 pkt x 512 B and
// 3977 pkt x 1024 B produced identical late%). That makes the transmit ceiling a
// packets-per-second question, which is what the TX task here measures.
//
// Deliberately NOT started at boot. Bringing ethernet up is a console command
// ('eth') so the same binary yields a true no-ethernet baseline without a
// reflash — reflashing between runs would change the comparison as much as the
// network does.
//
// RX and TX are SEPARATE tasks on purpose. An earlier version paced transmission
// from inside the receive loop, which capped TX at the recv timeout rate and made
// the measured send rate meaningless as a ceiling.
//
// UDP rather than TCP: RTPS/DDS is UDP, and TCP's congestion control would mask
// the effect being measured.

#include <atomic>
#include <cstdint>
#include <cstring>

#include <esp_timer.h>
#include <lwip/sockets.h>

#include "format.hpp"
#include "logger.hpp"
#include "pace-racer-board.hpp"

namespace netload {

// Point-to-point link through the USB dock: no DHCP server exists on this
// segment, so both ends are statically addressed.
//
// A dedicated subnet rather than 169.254/16 link-local, for two reasons learned
// the hard way: Tailscale installs a route for the whole link-local range that
// outranks the physical interface and silently blackholes the traffic, and macOS
// re-rolls its self-assigned link-local address on every reconfigure (it moved
// three times in one session, breaking the host scripts each time).
//
// Host side, address only — deliberately NO router argument. The dock interface
// sits above Wi-Fi in macOS's service order, so configuring a gateway here would
// install a higher-priority default route and cut the host's internet:
//   sudo ifconfig en7 inet 192.168.50.1 netmask 255.255.255.0
static constexpr const char *kStaticIp = "192.168.50.50";
static constexpr const char *kNetmask = "255.255.255.0";
static constexpr uint16_t kPort = 3333;

// Sentinel for "transmit as fast as the stack will accept", used to find the
// ceiling rather than to hold a set rate.
static constexpr uint32_t kTxUnpaced = 0xFFFFFFFFu;

// Counters are relaxed atomics: diagnostics, not control inputs.
inline std::atomic<uint32_t> g_rx_pkts{0};
inline std::atomic<uint32_t> g_rx_bytes{0};
inline std::atomic<uint32_t> g_tx_pkts{0};
inline std::atomic<uint32_t> g_tx_bytes{0};
inline std::atomic<uint32_t> g_tx_errs{0};

inline std::atomic<uint32_t> g_tx_hz{0};    // 0 = idle, kTxUnpaced = flat out
inline std::atomic<uint32_t> g_tx_len{256}; // payload bytes per datagram

// Seconds of flat-out transmission to run, then stop and self-report. The burst
// MUST be bounded and self-terminating: the TX task is priority 5 on CPU0 and
// the console loop lives in app_main at priority 1 on the same core, so while
// this task floods it starves the console — including the 'etx 0' that would
// otherwise stop it. Learned the hard way; an unbounded flood needs a reflash to
// escape.
inline std::atomic<uint32_t> g_burst_secs{0};
inline std::atomic<bool> g_running{false};
inline std::atomic<bool> g_tx_started{false};
inline std::atomic<bool> g_have_peer{false};

// Learned from the first datagram received. Publishing to whoever last spoke to
// us avoids hardcoding the host address, which matters because the host's
// link-local address is self-assigned and can change between runs.
inline sockaddr_in g_peer{};

inline void reset_counters() {
  g_rx_pkts.store(0, std::memory_order_relaxed);
  g_rx_bytes.store(0, std::memory_order_relaxed);
  g_tx_pkts.store(0, std::memory_order_relaxed);
  g_tx_bytes.store(0, std::memory_order_relaxed);
  g_tx_errs.store(0, std::memory_order_relaxed);
}

// ---------------------------------------------------------------- receive ----

inline void rx_task_fn(void *arg) {
  auto *logger = static_cast<espp::Logger *>(arg);

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    logger->error("netload: rx socket() failed, errno {}", errno);
    g_running.store(false);
    vTaskDelete(nullptr);
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(kPort);
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    logger->error("netload: bind() failed, errno {}", errno);
    close(sock);
    g_running.store(false);
    vTaskDelete(nullptr);
    return;
  }

  // Block until a packet arrives rather than polling. A short unconditional
  // timeout would wake this task thousands of times a second doing nothing,
  // which is itself load on the system being measured.
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100 ms
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  static uint8_t rxbuf[1500];
  logger->info("netload: UDP sink listening on port {}", kPort);

  while (g_running.load(std::memory_order_relaxed)) {
    sockaddr_in peer{};
    socklen_t plen = sizeof(peer);
    int n = recvfrom(sock, rxbuf, sizeof(rxbuf), 0, reinterpret_cast<sockaddr *>(&peer), &plen);
    if (n > 0) {
      g_rx_pkts.fetch_add(1, std::memory_order_relaxed);
      g_rx_bytes.fetch_add((uint32_t)n, std::memory_order_relaxed);
      if (!g_have_peer.load(std::memory_order_relaxed)) {
        g_peer = peer;
        g_have_peer.store(true, std::memory_order_release);
      }
    }
  }

  close(sock);
  logger->info("netload: sink stopped");
  vTaskDelete(nullptr);
}

// --------------------------------------------------------------- transmit ----

inline void tx_task_fn(void *arg) {
  auto *logger = static_cast<espp::Logger *>(arg);

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    logger->error("netload: tx socket() failed, errno {}", errno);
    vTaskDelete(nullptr);
    return;
  }

  static uint8_t txbuf[1500];
  memset(txbuf, 0xA5, sizeof(txbuf));
  int64_t next_tx_us = esp_timer_get_time();

  int64_t burst_end = 0, burst_t0 = 0;
  uint32_t burst_p0 = 0, burst_b0 = 0;

  while (g_running.load(std::memory_order_relaxed)) {
    // Arm a bounded flat-out burst if one was requested.
    const uint32_t bs = g_burst_secs.exchange(0, std::memory_order_relaxed);
    if (bs > 0) {
      burst_t0 = esp_timer_get_time();
      burst_end = burst_t0 + (int64_t)bs * 1000000;
      burst_p0 = g_tx_pkts.load(std::memory_order_relaxed);
      burst_b0 = g_tx_bytes.load(std::memory_order_relaxed);
      g_tx_hz.store(kTxUnpaced, std::memory_order_relaxed);
    }
    if (burst_end != 0 && esp_timer_get_time() >= burst_end) {
      g_tx_hz.store(0, std::memory_order_relaxed);
      const double dt = (double)(esp_timer_get_time() - burst_t0) / 1e6;
      const uint32_t dp = g_tx_pkts.load(std::memory_order_relaxed) - burst_p0;
      const uint32_t db = g_tx_bytes.load(std::memory_order_relaxed) - burst_b0;
      burst_end = 0;
      fmt::print("#etxmax len={} pkts={} bytes={} in {:.2f}s -> {:.0f} pps {:.1f} kB/s err={}\n",
                 g_tx_len.load(), dp, db, dt, dp / dt, (double)db / 1024.0 / dt,
                 g_tx_errs.load());
    }

    const uint32_t hz = g_tx_hz.load(std::memory_order_relaxed);
    if (hz == 0 || !g_have_peer.load(std::memory_order_acquire)) {
      vTaskDelay(pdMS_TO_TICKS(20)); // idle: yield, do not spin
      next_tx_us = esp_timer_get_time();
      continue;
    }

    if (hz != kTxUnpaced) {
      const int64_t now = esp_timer_get_time();
      const int64_t interval = 1000000 / (int64_t)hz;
      if (now < next_tx_us) {
        // Sub-tick waits cannot be expressed to the scheduler; busy-yield for
        // those, and actually sleep when the gap is worth a context switch.
        const int64_t wait_us = next_tx_us - now;
        if (wait_us > 2000) {
          vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
        } else {
          taskYIELD();
        }
        continue;
      }
      // Catch up rather than drift, but resync instead of emitting a backlog
      // burst, which would itself be a load spike.
      next_tx_us = (now - next_tx_us > interval) ? now + interval : next_tx_us + interval;
    }

    uint32_t len = g_tx_len.load(std::memory_order_relaxed);
    if (len > sizeof(txbuf)) len = sizeof(txbuf);
    int sent = sendto(sock, txbuf, len, 0, reinterpret_cast<sockaddr *>(&g_peer), sizeof(g_peer));
    if (sent > 0) {
      g_tx_pkts.fetch_add(1, std::memory_order_relaxed);
      g_tx_bytes.fetch_add((uint32_t)sent, std::memory_order_relaxed);
    } else {
      g_tx_errs.fetch_add(1, std::memory_order_relaxed);
      // ENOMEM/ENOBUFS just means the stack is saturated, which is exactly what
      // the unpaced ceiling test is trying to find. Yield and keep going.
      taskYIELD();
    }
  }

  close(sock);
  vTaskDelete(nullptr);
}

/// Bring up ethernet and start the RX sink and TX tasks. Idempotent.
inline bool start(espp::PaceRacerBoard &bsp, espp::Logger &logger) {
  if (g_running.load(std::memory_order_relaxed)) {
    return true;
  }

  espp::PaceRacerBoard::EthernetConfig cfg;
  cfg.hostname = "pace-racer";
  cfg.use_dhcp = false;
  cfg.static_ip = kStaticIp;
  cfg.netmask = kNetmask;
  cfg.gateway = "";

  std::error_code ec;
  if (!bsp.init_ethernet(cfg, ec)) {
    logger.error("netload: ethernet init failed: {}", ec.message());
    return false;
  }

  reset_counters();
  g_running.store(true, std::memory_order_relaxed);

  // CPU0, priority 5: the FOC task is priority 20 pinned to CPU1, so networking
  // deliberately loses any contest for the CPU. If the control loop still
  // degrades, the coupling is the shared SPI2 bus or the W5500 IRQ rather than
  // scheduling — and that distinction is the point of the measurement.
  xTaskCreatePinnedToCore(rx_task_fn, "netrx", 4096, &logger, 5, nullptr, 0);
  xTaskCreatePinnedToCore(tx_task_fn, "nettx", 4096, &logger, 5, nullptr, 0);
  g_tx_started.store(true, std::memory_order_relaxed);
  return true;
}

inline void stop() { g_running.store(false, std::memory_order_relaxed); }

} // namespace netload
