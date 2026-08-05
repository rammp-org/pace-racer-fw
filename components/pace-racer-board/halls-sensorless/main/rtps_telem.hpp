#pragma once

// RTPS telemetry publisher — carries the existing 10 Hz CSV stream over DDS/RTPS
// instead of the USB console.
//
// GOAL: match the current telemetry level (10 Hz x 16 fields) over RTPS. That is
// deliberately a modest target against the measured transport budget: the W5500
// transmit ceiling is ~250 packets/s at every payload size, so 10 Hz is roughly
// 4% of it. The interesting risks are therefore NOT bandwidth — they are whether
// SPDP/SEDP discovery works on a point-to-point link, what RTPS costs per sample
// in CPU, and whether its tasks disturb the 20 kHz control loop.
//
// The payload is the same 16 values the CSV stream prints, so USB and RTPS can be
// compared like for like. As CDR that is ~64 B versus ~100 B of ASCII.
//
// Task placement matters here, and not in the obvious way. Measured 2026-08-03:
// the sampling ISR runs on CPU0 (registered from app_main, which is CPU0-pinned,
// and ESP-IDF allocates interrupts on the calling core), so piling work onto CPU0
// is what disturbs acquisition. Leaving these tasks UNPINNED measured better than
// pinning them to CPU0. They also get a real priority rather than espp's default
// of 0 — the console stream task sits at 0 and demonstrably starves under load.

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <esp_timer.h>

#include "cdr.hpp"
#include "format.hpp"
#include "logger.hpp"
#include "rtps.hpp"
#include "task.hpp"

namespace rtpstelem {

/// One telemetry sample — the same fields the CSV stream prints.
struct Sample {
  float seconds{};
  float id{}, iq{}, iqref{};
  float vd{}, vq{};
  float aerr{};
  float rpm_drive{}, rpm_est{}, rpm_hall{};
  float flux{};
  float temps[4]{};
  uint8_t state{};
};

using SampleFn = std::function<Sample()>;

inline std::atomic<uint32_t> g_pub_ok{0};
inline std::atomic<uint32_t> g_pub_fail{0};
inline std::atomic<uint32_t> g_participants{0};
inline std::atomic<uint32_t> g_endpoints{0};
inline std::atomic<uint32_t> g_rate_hz{10}; // 10 Hz == the current CSV stream rate
inline std::atomic<bool> g_running{false};
inline std::atomic<bool> g_started{false};

// Per-cycle publish cost, us. The publish rate saturates at 1/publish_time once
// the requested period drops below it, so this is the number that sets the
// ceiling — worth measuring directly rather than inferring from achieved rate.
inline std::atomic<uint32_t> g_pub_us_min{0xFFFFFFFFu};
inline std::atomic<uint32_t> g_pub_us_max{0};
inline std::atomic<uint64_t> g_pub_us_sum{0};
inline std::atomic<uint32_t> g_pub_us_n{0};

// Cycles where the deadline had already passed before we got to sleep — i.e. the
// publisher could not keep up with the requested rate.
inline std::atomic<uint32_t> g_pub_overrun{0};

// Reader-side receive counters, for measuring what inbound RTPS processing costs.
inline std::atomic<uint32_t> g_rx_samples{0};
inline std::atomic<uint32_t> g_rx_bytes{0};

inline std::atomic<uint32_t> g_num_writers{1};
inline std::atomic<uint32_t> g_num_readers{0};

inline std::unique_ptr<espp::RtpsParticipant> g_participant;
inline std::unique_ptr<espp::Task> g_pub_task;
inline SampleFn g_sample_fn;
inline std::vector<std::string> g_topics;

static constexpr const char *kTopic = "pace_racer/telemetry";
// Advertised as a plain CDR struct. ROS 2 name mangling is not implemented by the
// component, so this will not appear as a ROS 2 topic without extra work — that
// is out of scope for a transport-performance test.
static constexpr const char *kType = "pace_racer::msg::Telemetry";

/// Serialize a sample to encapsulated CDR. All members are <= 4-byte aligned, so
/// the alignment caveat about body-only writers does not apply.
inline std::vector<uint8_t> encode(const Sample &s) {
  espp::CdrWriter w; // little-endian, with encapsulation header
  w.write<float>(s.seconds);
  w.write<uint8_t>(s.state);
  w.write<float>(s.id);
  w.write<float>(s.iq);
  w.write<float>(s.iqref);
  w.write<float>(s.vd);
  w.write<float>(s.vq);
  w.write<float>(s.aerr);
  w.write<float>(s.rpm_drive);
  w.write<float>(s.rpm_est);
  w.write<float>(s.rpm_hall);
  w.write<float>(s.flux);
  for (float t : s.temps) {
    w.write<float>(t);
  }
  return w.take_buffer();
}

inline void reset_counters() {
  g_pub_ok.store(0, std::memory_order_relaxed);
  g_pub_fail.store(0, std::memory_order_relaxed);
  g_pub_us_min.store(0xFFFFFFFFu, std::memory_order_relaxed);
  g_pub_us_max.store(0, std::memory_order_relaxed);
  g_pub_us_sum.store(0, std::memory_order_relaxed);
  g_pub_us_n.store(0, std::memory_order_relaxed);
  g_pub_overrun.store(0, std::memory_order_relaxed);
  g_rx_samples.store(0, std::memory_order_relaxed);
  g_rx_bytes.store(0, std::memory_order_relaxed);
}

/// Bring up the RTPS participant and start publishing.
/// \param local_ip The board's address. MUST be the real interface address —
///        the component defaults advertised_address to 127.0.0.1, which peers
///        would then try to reach.
inline bool start(const std::string &local_ip, SampleFn sample_fn, espp::Logger &logger,
                  uint32_t n_writers = 1, uint32_t n_readers = 0) {
  if (g_started.load(std::memory_order_relaxed)) {
    return true;
  }
  g_sample_fn = std::move(sample_fn);
  if (n_writers < 1) n_writers = 1;
  if (n_writers > 16) n_writers = 16;
  if (n_readers > 16) n_readers = 16;
  g_num_writers.store(n_writers, std::memory_order_relaxed);
  g_num_readers.store(n_readers, std::memory_order_relaxed);

  espp::RtpsParticipant::Config cfg;
  cfg.node_name = "pace_racer";
  cfg.domain_id = 0;
  cfg.participant_id = 0;
  cfg.bind_address = "0.0.0.0";
  cfg.advertised_address = local_ip;
  // Publish user data to the multicast group rather than unicasting to matched
  // readers. Two reasons:
  //  - With no matched reader, build_user_send_configs() returns no destinations
  //    and publish() fails, so nothing reaches the wire and there is nothing to
  //    measure. Multicast adds a destination unconditionally.
  //  - Telemetry is a broadcast by nature; multicast is the natural fit and does
  //    not multiply packets per subscriber, which matters when the transport
  //    budget is ~250 packets/s.
  cfg.use_multicast_for_user_data = true;
  cfg.announce_period = std::chrono::milliseconds(1000);
  cfg.log_level = espp::Logger::Verbosity::WARN;

  // Unpinned, priority 5. See the header comment: unpinned beat CPU0-pinned in a
  // controlled A/B/C, and priority 0 (espp's default) starves under load.
  cfg.receive_task_config = {.name = "RtpsRx", .stack_size_bytes = 6 * 1024, .priority = 5,
                             .core_id = -1};
  cfg.announce_task_config = {.name = "RtpsAnnounce", .stack_size_bytes = 6 * 1024, .priority = 5,
                              .core_id = -1};
  cfg.heartbeat_task_config = {.name = "RtpsHeartbeat", .stack_size_bytes = 6 * 1024,
                               .priority = 5, .core_id = -1};

  cfg.on_participant_discovered = [](const espp::RtpsParticipant::ParticipantProxy &p) {
    g_participants.fetch_add(1, std::memory_order_relaxed);
    fmt::print("#rtps discovered participant '{}' at {}\n", p.name, p.address);
  };
  cfg.on_endpoint_discovered = [](const espp::RtpsParticipant::EndpointProxy &) {
    g_endpoints.fetch_add(1, std::memory_order_relaxed);
  };

  g_participant = std::make_unique<espp::RtpsParticipant>(cfg);

  // One writer per topic. Multiple writers measure how per-endpoint cost scales:
  // each publish() does its own lookup, message build, and send.
  g_topics.clear();
  for (uint32_t i = 0; i < n_writers; i++) {
    espp::RtpsParticipant::WriterConfig wcfg;
    wcfg.topic_name = (n_writers == 1) ? kTopic : fmt::format("{}{}", kTopic, i);
    wcfg.type_name = kType;
    wcfg.entity_index = i;
    wcfg.history_depth = 4; // shallow: telemetry is live, stale samples are useless
    if (!g_participant->add_writer(wcfg)) {
      logger.error("rtps: add_writer {} failed", i);
      g_participant.reset();
      return false;
    }
    g_topics.push_back(wcfg.topic_name);
  }

  // Readers subscribed to the same topics. With multicast user data the board
  // receives its own publications, so this exercises the full inbound path
  // (parse, route by writer GUID, deliver) without needing a second node.
  for (uint32_t i = 0; i < n_readers; i++) {
    espp::RtpsParticipant::ReaderConfig rcfg;
    rcfg.topic_name = g_topics[i % g_topics.size()];
    rcfg.type_name = kType;
    rcfg.entity_index = i;
    rcfg.multicast_group = cfg.user_multicast_group;
    rcfg.on_sample = [](std::span<const uint8_t> data) {
      g_rx_samples.fetch_add(1, std::memory_order_relaxed);
      g_rx_bytes.fetch_add((uint32_t)data.size(), std::memory_order_relaxed);
    };
    if (!g_participant->add_reader(rcfg)) {
      logger.error("rtps: add_reader {} failed", i);
      g_participant.reset();
      return false;
    }
  }

  if (!g_participant->start()) {
    logger.error("rtps: participant start failed");
    g_participant.reset();
    return false;
  }

  reset_counters();
  g_running.store(true, std::memory_order_relaxed);

  g_pub_task = espp::Task::make_unique(espp::Task::Config{
      .callback =
          [](std::mutex &m, std::condition_variable &cv) -> bool {
            // Absolute deadline, NOT a relative sleep. cv.wait_for(period) after
            // doing the work makes the true cycle (work + period), which at 10 Hz
            // with ~12 ms of publish work yields 8.9 Hz, not 10. Advancing a
            // deadline by exactly one period removes that drift. (The CSV stream
            // task in sensorless.cpp has the same flaw and the same 8.9 Hz
            // symptom.)
            static auto next = std::chrono::steady_clock::now();

            if (!g_running.load(std::memory_order_relaxed)) {
              return true; // stop the task
            }
            const uint32_t hz = g_rate_hz.load(std::memory_order_relaxed);
            if (hz == 0) {
              next = std::chrono::steady_clock::now();
              std::unique_lock<std::mutex> lk(m);
              cv.wait_for(lk, std::chrono::milliseconds(20));
              return false;
            }

            if (g_sample_fn && g_participant) {
              const int64_t t0 = esp_timer_get_time();
              auto payload = encode(g_sample_fn());
              for (const auto &topic : g_topics) {
                if (g_participant->publish(topic, payload)) {
                  g_pub_ok.fetch_add(1, std::memory_order_relaxed);
                } else {
                  g_pub_fail.fetch_add(1, std::memory_order_relaxed);
                }
              }
              const uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
              g_pub_us_sum.fetch_add(dt, std::memory_order_relaxed);
              g_pub_us_n.fetch_add(1, std::memory_order_relaxed);
              uint32_t prev = g_pub_us_max.load(std::memory_order_relaxed);
              while (dt > prev &&
                     !g_pub_us_max.compare_exchange_weak(prev, dt, std::memory_order_relaxed)) {
              }
              prev = g_pub_us_min.load(std::memory_order_relaxed);
              while (dt < prev &&
                     !g_pub_us_min.compare_exchange_weak(prev, dt, std::memory_order_relaxed)) {
              }
            }

            const auto period = std::chrono::microseconds(1000000 / (hz > 2000 ? 2000 : hz));
            next += period;
            const auto now = std::chrono::steady_clock::now();
            if (next <= now) {
              // Could not keep up: resync rather than accumulate a backlog that
              // would then be emitted as a burst.
              g_pub_overrun.fetch_add(1, std::memory_order_relaxed);
              next = now;
              return false;
            }
            std::unique_lock<std::mutex> lk(m);
            cv.wait_until(lk, next);
            return false;
          },
      .task_config = {.name = "RtpsPub", .stack_size_bytes = 6 * 1024, .priority = 5,
                      .core_id = -1},
      .log_level = espp::Logger::Verbosity::WARN,
  });
  g_pub_task->start();

  g_started.store(true, std::memory_order_relaxed);
  logger.info("rtps: participant up on {} publishing '{}'", local_ip, kTopic);
  return true;
}

inline void stop() {
  g_running.store(false, std::memory_order_relaxed);
  if (g_pub_task) {
    g_pub_task->stop();
    g_pub_task.reset();
  }
  if (g_participant) {
    g_participant->stop();
    g_participant.reset();
  }
  g_started.store(false, std::memory_order_relaxed);
}

} // namespace rtpstelem
