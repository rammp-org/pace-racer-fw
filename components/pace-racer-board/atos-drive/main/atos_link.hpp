#pragma once

// The board's end of the ATOS wire: DHCP ethernet, one RTPS participant, one
// MotorCommand reader on this axis's command topic, one MotorState writer on its
// state topic at 20 Hz.
//
// Lifecycle follows the IP address. The BSP's ethernet callbacks run on the
// event-loop task and must not block, so they only record the new address; the
// link task (20 Hz, priority 5, unpinned, per the coexistence measurements)
// notices the change and starts or stops the participant itself. espp cannot
// auto-detect the interface on ESP targets, so the participant is created with
// the leased address and recreated on a new lease.
//
// The reader callback runs on the engine's receive thread. It hands the command
// to the supervisor and returns; every control decision is the supervisor's, on
// its own tick. The publish task uses the absolute-deadline pattern from the
// telemetry experiment so 20 Hz means 20 Hz, not 20 Hz minus publish time.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "logger.hpp"
#include "pace-racer-board.hpp"
#include "rtps_interface.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "task.hpp"

namespace atoslink {

using CommandFn = std::function<void(const rammp::MotorCommand &)>;
using StateFn = std::function<rammp::MotorState()>;

struct Config {
  rammp::AxisId axis{rammp::AxisId::UNASSIGNED};
  CommandFn on_command;    ///< called on the RTPS receive thread; must be quick
  StateFn state_fn;        ///< sampled on the link task at publish time
  std::string hostname{"pace-racer"};
  uint32_t publish_hz{20};
};

struct Counters {
  std::atomic<uint32_t> rx{0};         ///< MotorCommands received
  std::atomic<uint32_t> tx_ok{0};      ///< MotorStates published
  std::atomic<uint32_t> tx_fail{0};    ///< publish() returned false (no matched reader is one cause)
  std::atomic<uint32_t> overrun{0};    ///< publish deadline already passed
  std::atomic<uint32_t> restarts{0};   ///< participant (re)starts
  std::atomic<uint32_t> cmd_matched{0}; ///< remote writer found for our command reader
  std::atomic<uint32_t> state_matched{0}; ///< remote reader found for our state writer
};

class Link {
public:
  explicit Link(Config cfg)
      : cfg_(std::move(cfg)) {}

  /// Bring up ethernet (DHCP) and the link task. Returns false only if the
  /// W5500 could not be installed; a missing lease is not an error, the task
  /// waits for it.
  bool start(espp::PaceRacerBoard &bsp, espp::Logger &logger) {
    logger_ = &logger;
    if (cfg_.axis == rammp::AxisId::UNASSIGNED) {
      logger.warn("atos: axis unassigned; ethernet up, RTPS stays down ('axis <n>')");
    }
    espp::PaceRacerBoard::EthernetConfig ecfg;
    ecfg.hostname = cfg_.hostname;
    ecfg.use_dhcp = true;
    ecfg.on_got_ip = [this](const std::string &ip) {
      std::lock_guard<std::mutex> lk(ip_mutex_);
      ip_ = ip;
      ip_changed_.store(true, std::memory_order_release);
    };
    ecfg.on_ip_lost = [this]() {
      std::lock_guard<std::mutex> lk(ip_mutex_);
      ip_.clear();
      ip_changed_.store(true, std::memory_order_release);
    };
    std::error_code ec;
    if (!bsp.init_ethernet(ecfg, ec)) {
      logger.error("atos: ethernet init failed: {}", ec.message());
      return false;
    }
    task_ = espp::Task::make_unique(espp::Task::Config{
        .callback = [this](std::mutex &m, std::condition_variable &cv) { return tick(m, cv); },
        .task_config = {.name = "AtosLink", .stack_size_bytes = 8 * 1024, .priority = 5, .core_id = -1},
        .log_level = espp::Logger::Verbosity::WARN,
    });
    task_->start();
    return true;
  }

  void stop() {
    if (task_) {
      task_->stop();
      task_.reset();
    }
    tear_down();
  }

  bool up() const { return participant_ != nullptr; }
  std::string ip() const {
    std::lock_guard<std::mutex> lk(ip_mutex_);
    return ip_;
  }
  const Counters &counters() const { return counters_; }

private:
  bool tick(std::mutex &m, std::condition_variable &cv) {
    static auto next = std::chrono::steady_clock::now();

    if (ip_changed_.exchange(false, std::memory_order_acq_rel)) {
      tear_down();
      const std::string ip = this->ip();
      if (!ip.empty() && cfg_.axis != rammp::AxisId::UNASSIGNED) {
        bring_up(ip);
      }
      next = std::chrono::steady_clock::now();
    }

    if (publisher_ && cfg_.state_fn) {
      if (publisher_->publish(cfg_.state_fn())) {
        counters_.tx_ok.fetch_add(1, std::memory_order_relaxed);
      } else {
        counters_.tx_fail.fetch_add(1, std::memory_order_relaxed);
      }
    }

    const auto period = std::chrono::microseconds(1000000 / (cfg_.publish_hz ? cfg_.publish_hz : 20));
    next += period;
    const auto now = std::chrono::steady_clock::now();
    if (next <= now) {
      counters_.overrun.fetch_add(1, std::memory_order_relaxed);
      next = now; // resync rather than burst
      return false;
    }
    std::unique_lock<std::mutex> lk(m);
    cv.wait_until(lk, next);
    return false;
  }

  void bring_up(const std::string &ip) {
    espp::RtpsParticipant::Config pcfg;
    pcfg.interface_address = ip;
    pcfg.on_publisher_matched = [this]() {
      counters_.state_matched.fetch_add(1, std::memory_order_relaxed);
    };
    pcfg.on_subscriber_matched = [this]() {
      counters_.cmd_matched.fetch_add(1, std::memory_order_relaxed);
    };
    pcfg.log_level = espp::Logger::Verbosity::WARN;
    participant_ = std::make_unique<espp::RtpsParticipant>(pcfg);
    if (!participant_->start()) {
      logger_->error("atos: participant start failed on {}", ip);
      participant_.reset();
      return;
    }
    const auto &axis = rammp::axis(cfg_.axis);
    publisher_ = std::make_unique<espp::Publisher<rammp::MotorState>>(
        *participant_,
        espp::Publisher<rammp::MotorState>::Config{.topic = axis.state.name,
                                                   .type_name = axis.state.type});
    subscriber_ = std::make_unique<espp::Subscriber<rammp::MotorCommand>>(
        *participant_,
        espp::Subscriber<rammp::MotorCommand>::Config{
            .topic = axis.command.name,
            .type_name = axis.command.type,
            .on_message =
                [this](const rammp::MotorCommand &c) {
                  counters_.rx.fetch_add(1, std::memory_order_relaxed);
                  if (cfg_.on_command)
                    cfg_.on_command(c);
                },
        });
    if (!publisher_->is_valid() || !subscriber_->is_valid()) {
      logger_->error("atos: endpoint registration failed");
      tear_down();
      return;
    }
    counters_.restarts.fetch_add(1, std::memory_order_relaxed);
    logger_->info("atos: up on {} as {} (cmd '{}', state '{}')", ip, axis.segment,
                  axis.command.name, axis.state.name);
  }

  void tear_down() {
    if (participant_) {
      // Stop first: reader callbacks live on the participant until it stops.
      participant_->stop();
    }
    subscriber_.reset();
    publisher_.reset();
    participant_.reset();
  }

  Config cfg_;
  espp::Logger *logger_{nullptr};
  mutable std::mutex ip_mutex_;
  std::string ip_;
  std::atomic<bool> ip_changed_{false};
  std::unique_ptr<espp::RtpsParticipant> participant_;
  std::unique_ptr<espp::Publisher<rammp::MotorState>> publisher_;
  std::unique_ptr<espp::Subscriber<rammp::MotorCommand>> subscriber_;
  std::unique_ptr<espp::Task> task_;
  Counters counters_;
};

} // namespace atoslink
