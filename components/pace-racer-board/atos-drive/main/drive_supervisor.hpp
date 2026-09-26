#pragma once

// The board state machine of the ATOS contract (docs/ATOS_MOTOR_API.md,
// "Behavior contract"), between the wire and the FOC atomics.
//
//   DISARMED  Hi-Z.
//   ARMED     running `mode` on the setpoints.
//   SAFE_STOPPING  velocity ramping to zero under the torque limit.
//   HOLDING   safe stop finished, position held under kHoldAmps.
//   FAULT     latched until a NEW clear_fault_req_id; a WATCHDOG fault keeps
//             holding, every other fault is already coasting (the FOC guards
//             exit through the gentle unload -> Hi-Z path on their own).
//
// The supervisor never touches the FOC globals. It drives them through Hooks
// the app provides, and reads the plant through Hooks::measure. That keeps the
// contract testable against a fake plant and keeps every mapping (N.m -> A,
// rad/s -> rpm, the limit ceilings) in one place: apply_mode() below.
//
// Threads: on_command() runs on the RTPS receive thread and only stores the
// command under a mutex. tick() runs on the supervisor task at 100 Hz and makes
// every decision. state() is called by the link task at 20 Hz and reads the
// last MotorState tick() assembled, under the same mutex.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

#include <esp_timer.h>

#include "motor_params.hpp"
#include "rtps_interface.hpp"

namespace supervisor {

/// What the plant looks like right now, sampled by the app for each tick.
struct Measured {
  float position_rad{0};   ///< shaft, multiturn from boot, CCW positive
  float velocity_rad_s{0}; ///< shaft
  float iq{0}, id{0};      ///< A, rotor frame
  float vd{0}, vq{0};      ///< V, loop effort
  std::array<float, 4> temps{NAN, NAN, NAN, NAN};
  uint16_t drv_status{0};  ///< raw DRV8353 fault bits from the last poll
  bool coasting{false};    ///< gate driver is in Hi-Z
  bool sampler_enabled{true};
  bool fb_calibrated{true};  ///< the selected feedback source is calibrated; arming is refused without it
  /// Guard events since the last tick. The FOC side has already reacted
  /// (unload + coast); the supervisor only names the fault.
  bool soft_trip{false};   ///< phase current over the trip level
  bool thermal{false};     ///< a board temperature over its limit
  bool overrun{false};     ///< sampler persistently late, disabled itself
  bool drv_fault{false};   ///< DRV8353 reported a fault (drv_status has the bits)
  bool hall_fault{false};  ///< hall inputs illegal while driving (FOC side already stopped)
  bool stall{false};       ///< no motion at the torque cap for 0.5 s (FOC side already stopped)
};

struct Hooks {
  /// Hi-Z the bridge with zero targets. Idempotent.
  std::function<void()> coast;
  /// Release Hi-Z so a drive command takes effect. Idempotent.
  std::function<void()> uncoast;
  /// Direct torque current. `cap_amps` is the |iq| ceiling; `max_rpm` is the
  /// speed above which the drive rolls torque off in the direction of motion
  /// (the command's vel_limit, so it means the same thing in every mode).
  std::function<void(float iq_amps, float cap_amps, float max_rpm)> torque;
  /// Speed loop: setpoint rpm, setpoint ramp rpm/s, iq ceiling, hard speed
  /// cap rpm (torque rolls off above it, whatever the loop asks).
  std::function<void(float rpm, float ramp_rpm_s, float cap_amps, float max_rpm)> velocity;
  /// Position loop on the rotor accumulator: target deg, speed clamp rpm, iq ceiling.
  /// `fresh` = new target, re-seed the derivative.
  std::function<void(float rotor_deg, float max_rpm, float cap_amps, bool fresh)> position;
  /// Zero targets, loop idle, bridge left as is (the caller coasts if wanted).
  std::function<void()> idle;
  /// A mode change or stop began: re-seed whatever the outer loops need. Never
  /// the current-loop PI while driving.
  std::function<void()> reset_controllers;
  std::function<Measured()> measure;
};

class Supervisor {
public:
  explicit Supervisor(Hooks hooks, rammp::AxisId axis)
      : hooks_(std::move(hooks))
      , axis_(axis) {
    state_.api_version = RAMMP_MOTOR_API_VERSION;
    state_.axis_id = axis;
    state_.fw_version = motor::kFwVersion;
    state_.vbus_measured = 0;
    state_.vbus = 48.0f;
  }

  /// RTPS receive thread. Store and return.
  void on_command(const rammp::MotorCommand &c) {
    std::lock_guard<std::mutex> lk(mutex_);
    cmd_ = c;
    cmd_time_us_ = esp_timer_get_time();
    have_cmd_ = true;
  }

  /// A command stream is live: the console must not drive.
  bool stream_live() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return have_cmd_ && cmd_age_ms_locked() < motor::kWatchdogMs;
  }

  /// Latest MotorState, assembled by tick().
  rammp::MotorState state() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return state_;
  }

  rammp::BoardState board_state() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return state_.state;
  }

  /// A fault raised by the app itself (console-side checks). Latches like any other.
  void raise(rammp::FaultCode code) {
    std::lock_guard<std::mutex> lk(mutex_);
    enter_fault_locked(code, /*holding=*/false);
  }

  /// 100 Hz. Everything happens here.
  void tick() {
    const Measured m = hooks_.measure ? hooks_.measure() : Measured{};
    std::lock_guard<std::mutex> lk(mutex_);
    using rammp::BoardState;
    using rammp::ControlMode;
    using rammp::FaultCode;
    using rammp::RequestedState;

    // 1. Faults the plant already acted on. Name them; the bridge is coasting.
    if (m.soft_trip)
      enter_fault_locked(FaultCode::OVERCURRENT, false);
    else if (m.thermal)
      enter_fault_locked(FaultCode::OVERTEMP, false);
    else if (m.overrun)
      enter_fault_locked(FaultCode::SAMPLER, false);
    else if (m.hall_fault)
      enter_fault_locked(FaultCode::HALL, false);
    else if (m.stall)
      enter_fault_locked(FaultCode::STALL, false);
    else if (m.drv_fault)
      enter_fault_locked((m.drv_status & 0x0200u) ? FaultCode::VDS_OCP : FaultCode::DRV_FAULT,
                         false);

    // 2. Clear request: a NEW counter value while faulted -> DISARMED.
    if (have_cmd_ && cmd_.clear_fault_req_id != clear_ack_) {
      clear_ack_ = cmd_.clear_fault_req_id;
      if (state_.state == BoardState::FAULT) {
        fault_ = FaultCode::NONE;
        set_state_locked(BoardState::DISARMED);
        go_idle_coast_locked();
      }
    }

    // 3. Watchdog: only matters while energized. A DISARMED board with no
    //    master stays DISARMED; energizing it to "hold" would be the bug.
    const uint32_t age_ms = have_cmd_ ? cmd_age_ms_locked() : 0xFFFFu;
    const bool watchdog = have_cmd_ && age_ms >= motor::kWatchdogMs;
    if (watchdog && (state_.state == BoardState::ARMED || state_.state == BoardState::SAFE_STOPPING)) {
      if (state_.state == BoardState::ARMED)
        begin_safe_stop_locked(m);
      watchdog_pending_ = true;
    } else if (watchdog && state_.state == BoardState::HOLDING) {
      enter_fault_locked(FaultCode::WATCHDOG, /*holding=*/true);
    }

    // 4. Requested level, if the stream is live.
    if (have_cmd_ && !watchdog) {
      switch (cmd_.requested_state) {
      case RequestedState::ESTOP:
      case RequestedState::DISARMED:
        // The master asked for Hi-Z: honored in every state, a holding fault
        // included (the latch stays; only the hold is released).
        if (state_.state != BoardState::FAULT) {
          set_state_locked(BoardState::DISARMED);
        }
        fault_holding_ = false;
        go_idle_coast_locked();
        break;
      case RequestedState::SAFE_STOP:
        if (state_.state == BoardState::ARMED) {
          begin_safe_stop_locked(m);
        }
        break;
      case RequestedState::ARMED:
        if (state_.state == BoardState::DISARMED || state_.state == BoardState::HOLDING) {
          if (!m.fb_calibrated) {
            enter_fault_locked(FaultCode::ENCODER, false); // "feedback": halls or encoder
          } else if (!m.sampler_enabled) {
            enter_fault_locked(FaultCode::SAMPLER, false);
          } else {
            if (hooks_.uncoast)
              hooks_.uncoast();
            set_state_locked(BoardState::ARMED);
            prev_mode_ = std::nullopt; // first apply_mode resets the loops
          }
        }
        break;
      }
    }

    // 5. Run the state.
    switch (state_.state) {
    case BoardState::ARMED:
      apply_mode_locked(m);
      break;
    case BoardState::SAFE_STOPPING:
      if (hooks_.velocity)
        hooks_.velocity(0.0f, safe_stop_ramp_, safe_stop_cap_, vel_rpm_limit_locked());
      if (std::fabs(m.velocity_rad_s * motor::kRadSToRpm) < motor::kStoppedRpm) {
        hold_deg_ = m.position_rad * motor::kRadToDeg;
        set_state_locked(BoardState::HOLDING);
        if (hooks_.position)
          hooks_.position(hold_deg_, motor::kHoldMaxRpm, motor::kHoldAmps, true);
        if (watchdog_pending_) {
          watchdog_pending_ = false;
          enter_fault_locked(FaultCode::WATCHDOG, /*holding=*/true);
        }
      }
      break;
    case BoardState::HOLDING:
      if (hooks_.position)
        hooks_.position(hold_deg_, motor::kHoldMaxRpm, motor::kHoldAmps, false);
      break;
    case BoardState::FAULT:
      if (fault_holding_ && hooks_.position)
        hooks_.position(hold_deg_, motor::kHoldMaxRpm, motor::kHoldAmps, false);
      break;
    case BoardState::DISARMED:
      break;
    }

    // 6. Telemetry.
    state_.seq++;
    state_.last_cmd_seq = have_cmd_ ? cmd_.seq : 0;
    state_.cmd_age_ms = (uint16_t)std::min<uint32_t>(age_ms, 0xFFFFu);
    state_.fault_code = fault_;
    state_.clear_fault_ack = clear_ack_;
    state_.drv_status = m.drv_status;
    state_.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    state_.position = m.position_rad;
    state_.velocity = m.velocity_rad_s;
    state_.torque_est = motor::kKt * m.iq;
    state_.iq = m.iq;
    state_.id = m.id;
    state_.ibus_est = (m.vd * m.id + m.vq * m.iq) / state_.vbus;
    state_.temps = m.temps;
    switch (state_.state) {
    case BoardState::ARMED:
      state_.mode = have_cmd_ ? cmd_.mode : ControlMode::COAST;
      break;
    case BoardState::SAFE_STOPPING:
      state_.mode = ControlMode::VELOCITY;
      break;
    case BoardState::HOLDING:
      state_.mode = ControlMode::HOLD;
      break;
    case BoardState::FAULT:
      state_.mode = fault_holding_ ? ControlMode::HOLD : ControlMode::COAST;
      break;
    case BoardState::DISARMED:
      state_.mode = ControlMode::COAST;
      break;
    }
  }

private:
  uint32_t cmd_age_ms_locked() const {
    const int64_t dt = esp_timer_get_time() - cmd_time_us_;
    return dt < 0 ? 0u : (uint32_t)std::min<int64_t>(dt / 1000, 0xFFFFFFFF);
  }

  void set_state_locked(rammp::BoardState s) { state_.state = s; }

  void go_idle_coast_locked() {
    if (hooks_.idle)
      hooks_.idle();
    if (hooks_.coast)
      hooks_.coast();
    prev_mode_ = std::nullopt;
  }

  void enter_fault_locked(rammp::FaultCode code, bool holding) {
    if (state_.state == rammp::BoardState::FAULT)
      return; // first fault wins; the rest is in drv_status / the console
    fault_ = code;
    fault_holding_ = holding;
    set_state_locked(rammp::BoardState::FAULT);
    if (!holding)
      go_idle_coast_locked();
  }

  void begin_safe_stop_locked(const Measured &) {
    // Ramp under the command's accel limit, or the stop-path default. Torque
    // cap stays what the command allowed so the stop is as firm as the drive.
    const float accel = have_cmd_ ? cmd_.accel_limit : 0.0f;
    safe_stop_ramp_ = accel > 0 ? accel * motor::kRadSToRpm : motor::kSafeStopRampRpmS;
    safe_stop_cap_ = cap_amps_locked();
    set_state_locked(rammp::BoardState::SAFE_STOPPING);
    if (hooks_.reset_controllers)
      hooks_.reset_controllers();
  }

  float cap_amps_locked() {
    const float lim = have_cmd_ ? cmd_.torque_limit : 0.0f;
    const float ceiling = motor::kMaxAmps * motor::kKt;
    const float eff = (lim > 0 ? std::min(lim, ceiling) : ceiling);
    state_.torque_limit_eff = eff;
    return eff / motor::kKt;
  }

  float vel_rpm_limit_locked() {
    const float lim = have_cmd_ ? cmd_.vel_limit : 0.0f;
    const float ceiling = motor::kMaxRpm * motor::kRpmToRadS;
    const float eff = (lim > 0 ? std::min(lim, ceiling) : ceiling);
    state_.vel_limit_eff = eff;
    return eff * motor::kRadSToRpm;
  }

  float ramp_rpm_s_locked() {
    // 0 = step, unless the motor profile has an acceleration ceiling: then 0
    // means the ceiling and a larger request is clamped to it, like the other
    // limits. The reported effective value is what the drive will do.
    const float lim = have_cmd_ ? cmd_.accel_limit : 0.0f;
    const float ceiling = motor::kMaxAccelRpmS * motor::kRpmToRadS; // rad/s^2, 0 = none
    float eff = lim;
    if (ceiling > 0 && (lim <= 0 || lim > ceiling))
      eff = ceiling;
    state_.accel_limit_eff = eff > 0 ? eff : 0.0f;
    return eff > 0 ? eff * motor::kRadSToRpm : 1.0e6f;
  }

  void apply_mode_locked(const Measured &m) {
    using rammp::ControlMode;
    const ControlMode mode = cmd_.mode;
    const bool fresh = !prev_mode_ || *prev_mode_ != mode;
    if (fresh && hooks_.reset_controllers)
      hooks_.reset_controllers();
    prev_mode_ = mode;

    const float cap = cap_amps_locked();
    const float vmax = vel_rpm_limit_locked();
    const float ramp = ramp_rpm_s_locked();

    switch (mode) {
    case ControlMode::COAST:
      if (hooks_.idle)
        hooks_.idle();
      if (hooks_.coast)
        hooks_.coast();
      break;
    case ControlMode::TORQUE:
      if (hooks_.uncoast)
        hooks_.uncoast();
      if (hooks_.torque)
        hooks_.torque(std::clamp(cmd_.torque / motor::kKt, -cap, cap), cap, vmax);
      break;
    case ControlMode::VELOCITY:
      if (hooks_.uncoast)
        hooks_.uncoast();
      if (hooks_.velocity)
        hooks_.velocity(std::clamp(cmd_.velocity * motor::kRadSToRpm, -vmax, vmax), ramp, cap, vmax);
      break;
    case ControlMode::POSITION:
      if (hooks_.uncoast)
        hooks_.uncoast();
      if (hooks_.position)
        hooks_.position(cmd_.position * motor::kRadToDeg, vmax, cap, fresh);
      break;
    case ControlMode::HOLD:
      if (fresh)
        hold_deg_ = m.position_rad * motor::kRadToDeg;
      if (hooks_.uncoast)
        hooks_.uncoast();
      if (hooks_.position)
        hooks_.position(hold_deg_, std::min(vmax, motor::kHoldMaxRpm), cap, fresh);
      break;
    }
  }

  Hooks hooks_;
  rammp::AxisId axis_;
  mutable std::mutex mutex_;
  rammp::MotorCommand cmd_{};
  int64_t cmd_time_us_{0};
  bool have_cmd_{false};
  rammp::MotorState state_{};
  rammp::FaultCode fault_{rammp::FaultCode::NONE};
  bool fault_holding_{false};
  bool watchdog_pending_{false};
  uint8_t clear_ack_{0};
  std::optional<rammp::ControlMode> prev_mode_;
  float hold_deg_{0};
  float safe_stop_ramp_{motor::kSafeStopRampRpmS};
  float safe_stop_cap_{motor::kHoldAmps};
};

} // namespace supervisor
