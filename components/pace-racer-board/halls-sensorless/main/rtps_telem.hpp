#pragma once

// RTPS telemetry publisher — carries the existing 10 Hz CSV stream over DDS/RTPS
// instead of the USB console.
//
// GOAL: match the current telemetry level (10 Hz x 16 fields) over RTPS. That is
// deliberately a modest target against the measured transport budget, so the
// interesting risks are NOT bandwidth — they are whether discovery works on a
// point-to-point link, what RTPS costs per sample in CPU, and whether its tasks
// disturb the 20 kHz control loop.
//
// The payload is the same 16 values the CSV stream prints, so USB and RTPS can be
// compared like for like. As CDR that is ~64 B versus ~100 B of ASCII.
//
// espp v1.2.0 port: the participant facade was rewritten around the embeddedRTPS
// engine and a typed pub/sub layer (espp::Publisher<T> / espp::Subscriber<T>)
// whose (de)serialization is reflection-driven from the Sample struct — the
// manual CdrWriter encode() is gone, and so are the per-task placement knobs
// (the engine owns its threads now) and the discovery callbacks (replaced by
// per-endpoint matched callbacks). Endpoints are added AFTER the participant is
// started in this API. Wire layout is XCDR1 in Sample's declaration order —
// tests/rtps_sub.py decodes exactly that, keep the two in sync.

#include <array>
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
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "task.hpp"

namespace rtpstelem {

/// One telemetry sample — the same fields the CSV stream prints. A plain
/// reflectable struct: the cdr component derives the XCDR1 wire format from the
/// member order below (floats first, state last, temps as std::array — C arrays
/// are not reflectable).
struct Sample {
  float seconds{};
  float id{}, iq{}, iqref{};
  float vd{}, vq{};
  float aerr{};
  float rpm_drive{}, rpm_est{}, rpm_hall{};
  float flux{};
  std::array<float, 4> temps{};
  uint8_t state{};
};

using SampleFn = std::function<Sample()>;

inline std::atomic<uint32_t> g_pub_ok{0};
inline std::atomic<uint32_t> g_pub_fail{0};
inline std::atomic<uint32_t> g_pub_matched{0}; // a writer gained a remote reader
inline std::atomic<uint32_t> g_sub_matched{0}; // a reader gained a remote writer
inline std::atomic<uint32_t> g_rate_hz{10};    // 10 Hz == the current CSV stream rate
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

// Reader-side receive counters, for measuring what inbound RTPS processing
// costs. NOTE: with the v1.2.0 engine, user data is unicast to matched remote
// readers — a reader on this participant no longer sees the board's own
// publications the way the old multicast-user-data mode allowed, so driving
// these counters needs an external writer on the topic.
inline std::atomic<uint32_t> g_rx_samples{0};
inline std::atomic<uint32_t> g_rx_bytes{0};

inline std::atomic<uint32_t> g_num_writers{1};
inline std::atomic<uint32_t> g_num_readers{0};

inline std::unique_ptr<espp::RtpsParticipant> g_participant;
// Publisher holds a mutex (non-movable) — keep them behind unique_ptr.
inline std::vector<std::unique_ptr<espp::Publisher<Sample>>> g_publishers;
inline std::vector<std::unique_ptr<espp::Subscriber<Sample>>> g_subscribers;
inline std::unique_ptr<espp::Task> g_pub_task;
inline SampleFn g_sample_fn;

static constexpr const char *kTopic = "pace_racer/telemetry";
// Advertised as a plain CDR struct. For ROS 2 visibility this would need the
// "rt/" topic prefix and "<pkg>::msg::dds_::<Type>_" mangling (see
// espp::ros2::topic_name) — out of scope for a transport-performance test.
static constexpr const char *kType = "pace_racer::msg::Telemetry";

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
/// \param local_ip The board's interface address. Required on ESP targets —
///        the component cannot auto-detect it.
inline bool start(const std::string &local_ip, SampleFn sample_fn, espp::Logger &logger,
                  uint32_t n_writers = 1, uint32_t n_readers = 0) {
  if (g_started.load(std::memory_order_relaxed)) {
    return true;
  }
  g_sample_fn = std::move(sample_fn);
  if (n_writers < 1)
    n_writers = 1;
  if (n_writers > 16)
    n_writers = 16;
  if (n_readers > 16)
    n_readers = 16;
  g_num_writers.store(n_writers, std::memory_order_relaxed);
  g_num_readers.store(n_readers, std::memory_order_relaxed);

  espp::RtpsParticipant::Config cfg;
  cfg.interface_address = local_ip;
  cfg.on_publisher_matched = []() { g_pub_matched.fetch_add(1, std::memory_order_relaxed); };
  cfg.on_subscriber_matched = []() { g_sub_matched.fetch_add(1, std::memory_order_relaxed); };
  cfg.log_level = espp::Logger::Verbosity::WARN;

  g_participant = std::make_unique<espp::RtpsParticipant>(cfg);

  // v1.2.0: endpoints can only be added after a successful start().
  if (!g_participant->start()) {
    logger.error("rtps: participant start failed");
    g_participant.reset();
    return false;
  }

  // One typed publisher per topic. Multiple writers measure how per-endpoint
  // cost scales: each publish() does its own serialize, lookup, and send.
  // Telemetry is live data — BEST_EFFORT (the default): a lost sample is
  // worthless a cycle later, and reliable heartbeats would only add traffic.
  std::vector<std::string> topics;
  for (uint32_t i = 0; i < n_writers; i++) {
    const std::string topic = (n_writers == 1) ? kTopic : fmt::format("{}{}", kTopic, i);
    auto pub = std::make_unique<espp::Publisher<Sample>>(
        *g_participant, espp::Publisher<Sample>::Config{.topic = topic, .type_name = kType});
    if (!pub->is_valid()) {
      logger.error("rtps: publisher {} failed to register", i);
      g_publishers.clear();
      g_participant->stop();
      g_participant.reset();
      return false;
    }
    g_publishers.push_back(std::move(pub));
    topics.push_back(topic);
  }

  // Typed readers subscribed to the same topics. See the note on the rx
  // counters: they only count samples from an EXTERNAL writer on the topic;
  // with none matched they measure the cost of idle readers.
  for (uint32_t i = 0; i < n_readers; i++) {
    auto sub = std::make_unique<espp::Subscriber<Sample>>(
        *g_participant,
        espp::Subscriber<Sample>::Config{
            .topic = topics[i % topics.size()],
            .type_name = kType,
            .on_message =
                [](const Sample &s) {
                  g_rx_samples.fetch_add(1, std::memory_order_relaxed);
                  g_rx_bytes.fetch_add((uint32_t)cdr::serialized_size<cdr::xcdr1>(s),
                                       std::memory_order_relaxed);
                },
        });
    if (!sub->is_valid()) {
      logger.error("rtps: subscriber {} failed to register", i);
      g_subscribers.clear();
      g_publishers.clear();
      g_participant->stop();
      g_participant.reset();
      return false;
    }
    g_subscribers.push_back(std::move(sub));
  }

  reset_counters();
  g_running.store(true, std::memory_order_relaxed);

  g_pub_task = espp::Task::make_unique(espp::Task::Config{
      .callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
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

        if (g_sample_fn && !g_publishers.empty()) {
          const int64_t t0 = esp_timer_get_time();
          const Sample s = g_sample_fn();
          for (auto &pub : g_publishers) {
            if (pub->publish(s)) {
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
      // Unpinned, priority 5: unpinned beat CPU0-pinned in a controlled A/B/C
      // (the sampling ISR lives on CPU0), and priority 0 (espp's default)
      // starves under load. Applies to OUR publish task only — the engine's
      // internal threads are no longer configurable in v1.2.0.
      .task_config =
          {.name = "RtpsPub", .stack_size_bytes = 6 * 1024, .priority = 5, .core_id = -1},
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
    // Stop the participant BEFORE dropping the subscribers: reader callbacks
    // live on the participant until it stops (see espp::Subscriber docs).
    g_participant->stop();
  }
  g_subscribers.clear();
  g_publishers.clear();
  g_participant.reset();
  g_started.store(false, std::memory_order_relaxed);
}

} // namespace rtpstelem
