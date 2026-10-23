// atos-drive: the PACE RACER board as an ATOS (RTPS) motor controller.
//
// Fork of halls-sensorless (2026-09-16), kept whole: the bench-validated FOC
// sampler, current loop, hall/encoder commutation, and console live on
// unchanged below. What this app adds is the network side of the contract in
// rammp-rtps motor_message.hpp (see ../../../docs/ATOS_MOTOR_API.md):
//
//   axis_config.hpp      NVS: which wheel this board is, and its calibration
//   atos_link.hpp        DHCP ethernet, RTPS participant, MotorCommand in /
//                        MotorState out
//   drive_supervisor.hpp the state machine, watchdog, and the mapping from
//                        MotorCommand fields onto the FOC atomics below
//
// The console stays for provisioning, calibration and tuning. Its motion
// commands are refused while a MotorCommand stream is live so the board never
// has two masters. The ethernet-load and RTPS-telemetry experiments of the
// parent app (net_load.hpp, rtps_telem.hpp, 'eth'/'etx'/'rtps'...) are gone;
// their results are in ETHERNET_FOC_COEXISTENCE.md and the memory notes.
//
// ---- parent app history (halls-sensorless) ----------------------------------
//
// ---- stage 0 results (2026-07-21, 24 V bus) ----------------------------------
//   - Null vector is TEP (on_full): sampling at TEP read 3.56 A (matching a
//     clamp's 3.65 A and the async cross-check 3.45 A); TEZ read ~0. espp's
//     register_pwm_sample_callback docstring names the wrong edge.
//   - ~7.8 us/conversion, stable over millions of samples, after driving the
//     ADC HAL directly and hoisting adc_oneshot_hal_setup + REGI2C calibration
//     to init (adc_oneshot_read_isr re-ran that every call and tripped the WDT).
//   - GAIN_20 linear to at least 8.5 A; current scale ~2.8% low vs clamp (fine).
//   - No FPU in an ISR on Xtensa — the sampler is integer-only.
// See foc_sampler.hpp for how all of that is implemented.
//
// Console protocol (line-based, same shape as commanded-current):
//   d <duty>     common-mode duty on all three phases
//   v <volts>    differential A->B volts about the common mode
//   e <0|1>      sample edge: 0 = TEZ, 1 = TEP; also re-enables after a trip/overrun
//   n <N>        sample (and control) every Nth PWM period (1 = 20 kHz)
//   m <1|2>      conversions per sampling ISR: 1 alternates phases, 2 both in one window
//   p <x> <y>    manually sample phases x,y (0=A,1=B,2=C); disables auto-selection
//   p auto       re-enable automatic two-lowest-duty phase selection
//   z            re-run the zero-current calibration (outputs forced off)
//   a            async median-of-3 cross-check read (suspends the ISR sampler)
//   s            print and reset the ISR timing statistics
//   r            dump DRV8353 registers
//   f            outputs off
//
// Streams CSV at 25 Hz:
//   %time(s), dc0, u_v, ia, ib, ic, ialpha, ibeta, foc_khz, coalesce, isr_us

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <type_traits>

#include "driver/gpio.h"
#include "driver/mcpwm_timer.h"
#include "hal/mcpwm_ll.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "logger.hpp"
#include "pace-racer-board.hpp"
#include "task.hpp"

#include "fast_math.hpp" // espp::fast_sin / fast_cos

#include "foc_math.hpp"
#include "foc_observer.hpp"
#include "foc_sampler.hpp"
#include "hall_drive.hpp"

#include "atos_link.hpp"
#include "axis_config.hpp"
#include "drive_supervisor.hpp"
#include "motor_params.hpp"

#include "demo_link.hpp" // native USB: telemetry, console, OTA, reset, core dump
#include "eth_link.hpp"  // the same service stream over TCP: OTA, console, reset

using namespace std::chrono_literals;

static constexpr float kTwoPi = 6.28318530718f;

// Bring-up runs on 24 V, not 48 V: switching ripple scales with bus voltage and
// L is only 94 uH/phase, so halving the bus halves the ripple on a 10 A command.
// MUST match the bench supply: SVPWM scales duties by it, so the observer's
// commanded-voltage model is only right when this is the real bus voltage.
// (24 V was the bring-up value; 48 V doubles ripple to ~6-7 App on 94 uH —
// watch the LM75s. Re-zero with 'z' after changing supplies.)
static constexpr float kBusVoltage = motor::kBusVoltage;
// Lowered from 15 A during stage-3 bring-up: the I/f ramp was running the current
// away to ~22 A before the hardware OCP caught it. 10 A trips a runaway early
// (real commands are <=6 A + a few A of hunting overshoot) and spares the FETs.
// 15 A: above the lying-conversion creep band so phantom trips stop; real
// protection during bring-up is the scope + PSU current limit + DRV8353 VDS
// OCP (17-22 A, verified firing — fault 0x0620).
static constexpr float kTripAmps = 15.0f;
static constexpr float kMaxTargetAmps = 8.0f; // console clamp at boot; raise live with 'lim'

// High-power staircase ('lim', 'vl', 'vds', 'tl'): the escalation limits are
// runtime commands so a run doesn't need a reflash, bounded by these hard
// ceilings. The ordering that must survive every escalation step:
//   target < soft trip < plausibility gate (trip+3) < DRV VDS OCP < ADC rail.
// The '#range' line at zero-cal prints the actual ADC rail in amps (~±40 A at
// GAIN_20 with a centered zero) — the trip is blind past it.
static constexpr float kHardMaxTargetAmps = 30.0f; // raised for the 500 W campaign 2026-08-20
static constexpr float kHardMaxTripAmps = 45.0f;   // raised 35->45 2026-08-26: phantom phase-C spikes (clamp-verified <35 A real); DRV VDS OCP still backstops
static constexpr float kHardMaxVolts = motor::kMaxPhaseVolts; // 'vl' ceiling, 0.94 * vbus/sqrt(3),
                                                   // the SVPWM linear-region edge
// Board thermal guard: any LM75 above the limit while driving -> the same
// gentle STOPPING path as 'stop', with the data intact. Raised deliberately
// run-by-run with 'tl' to walk toward failure instead of ending in a bang.
// The LM75s see board copper, not FET junctions — the FLIR owns Tj; this
// catches the slow soak and the case where nobody is watching.
static constexpr float kTempLimitC = 80.0f;

// Stage 4: I/f -> observer handoff by CURRENT-DECAY CONVERGENCE (the "I-f with
// smooth transition" method). The synthetic angle is never touched; instead iq
// decays slowly, which forces the rotor's load angle toward 90 deg (torque
// balance: T = k*lam*I*sin(gamma), smaller I -> larger gamma), which rotates
// the rotor flux onto the synthetic d-axis. When the observer agrees the
// frames have converged (|th_est - th_drive| small), switching the angle
// source is a near-no-op — torque is continuous by construction. An angle
// CROSSFADE was tried first and failed on hardware: it swings the current
// vector toward max-torque orientation with iq unchanged, so torque-per-amp
// jumps ~5-10x mid-blend and the acceleration burst slipped the rotor.
static constexpr float kHoEngageRpm = 60.0f;
static constexpr float kHoRevertRpm = 45.0f;
static constexpr float kHoDecayAps = 1.0f; // iq decay rate during CONVERGE
// Absolute CONVERGE floor. A FRACTIONAL floor (25% of run amps) made
// convergence impossible at high run amps on the dyno: the load angle only
// reaches 90° when iq decays to ~the machine's actual drag current (sinγ =
// T/(k·λ·I)), and drag doesn't scale with the commanded amps — at `run 10` the
// 2.5 A floor parked aerr at ~90° until the stuck-abort fired, forever. 0.5 A
// matches the unloaded converge current observed on this rig.
static constexpr float kIqFloorAmps = 0.5f;
static constexpr float kConvergeDeg = 15.0f;       // frames agree inside this cone
static constexpr uint32_t kEngageTicks = 1000;     // 50 ms sustained above engage rpm
static constexpr uint32_t kConvergeTicks = 400;    // 20 ms inside the cone -> switch
static constexpr uint32_t kConvStuckTicks = 10000; // 500 ms at floor unconverged -> abort
static constexpr uint32_t kCooldownTicks = 10000;  // 500 ms after revert/abort: no chatter
// Outer speed loop, active only in CLOSED: rpm setpoint (ramped at the run's
// ramp rate) -> iq command, clamped to ±run amps (amps = torque ceiling).
// Few-Hz loop around a 400 rad/s current loop — decades of separation.
static constexpr float kSpeedKp = 0.05f; // A per rpm (encoder feedback, dyno tune)
static constexpr float kSpeedKi = 0.2f;  // A per rpm-second
// Hall feedback (2026-09-28). The planner streams setpoints and reads the
// velocity back. P alone cannot start a loaded wheel: the current it can ask
// for is Kp * setpoint, and 0.125 A/rpm at 30 rpm (3.75 A) did not break the
// rover loose. The integrator supplies the breakaway current; 2.0 wound
// 20 A/s per 10 rpm and was too lively, 0.5 builds 5 A in a second. Needs
// the 10 Hz tracking loop in hall_drive.hpp.
static constexpr float kHallSpeedKp = 0.25f;
static constexpr float kHallSpeedKi = 0.5f;
// Fail-safe: while the observer has authority, |rotor flux| leaving this band
// around lambda_pm for ~40 ms means the estimate is lost — coast, don't fight.
static constexpr float kFluxFailLo = 0.35f;
static constexpr float kFluxFailHi = 3.0f;
static constexpr uint32_t kFluxFailTicks = 800; // ~40 ms of control ticks
// Gentle stop: keep the loop CLOSED and unload to 0 A first (the PI integrator
// keeps cancelling back-EMF, so no transient), then request hardware Hi-Z
// coast. The old stop reset the PI while the bridge was live — the integrator's
// ~bemf worth of voltage vanished instantly and the bemf drove a current spike
// through 94 uH (seen on the clamp as a big phase-A kick at every stop).
static constexpr uint32_t kStopUnloadTicks = 2000; // ~100 ms at 20 kHz
// Never command 0 or 1: a zero comparator wedges the MCPWM generator, and duty 1
// leaves no null window. SVPWM centers duties at 0.5.
static constexpr float kMinDuty = 0.03f;
static constexpr float kMaxDuty = 0.97f;
static constexpr float kCenterDuty = 0.5f;   // SVPWM idle: all phases at 0.5 = no current
static constexpr float kPwmPeriodUs = 50.0f; // espp BldcDriver is fixed at 20 kHz

// dq current-loop voltage limit. The loop output is a voltage; this caps it so a
// bad command or gain can't slam the full bus. 8 V into ~0.15 ohm is ~50 A DC,
// but the loop is regulating current, so this is just the ceiling — start here.
static constexpr float kVoltageLimit = motor::kDefaultVoltLimit;

// Current-loop PI gains, per motor (motor_params.hpp). 169 rad/s bandwidth,
// ki = kp*R/L; the NineBot values are the auto-tuner's verified-overdamped
// crossover, the hub's are those scaled by its R and L.
static constexpr float kDefaultKp = motor::kDefaultKp; // V/A
static constexpr float kDefaultKi = motor::kDefaultKi; // V/(A*s)

// I/f open-loop startup (stage 2b). Pole pairs from motor_params.hpp.
static constexpr int kPolePairs = motor::kPolePairs;
static constexpr float kAlignMs = 300.0f; // DC-align time before the ramp
// Gentle first-spin defaults (override with `run <amps> <rpm> <ramp_s>`).
static constexpr float kSpinAmps = 2.0f;
static constexpr float kSpinRpm = 50.0f;
static constexpr float kSpinRampS = 2.0f;
// Mechanical rpm -> electrical rad/s.
static constexpr float kRpmToOmegaE = (float)kPolePairs * 6.28318530718f / 60.0f;

// Hall pins (validation ground truth, never in the control path).
static constexpr auto kHallA = GPIO_NUM_3;
static constexpr auto kHallB = GPIO_NUM_46;
static constexpr auto kHallC = GPIO_NUM_9;

// Flux-observer defaults. R and L from current-control plant ID (per-phase =
// half the A->B series fit): NineBot S = 0.323 ohm / 336 uH. lambda_pm seeded
// from vq at speed. All tunable from the console ('mp', 'og') — dial in from the
// logs. NOTE: kObsR ideally validated against the stage-2a vd=id*R slope.
static constexpr float kObsR = motor::kObsR;
static constexpr float kObsL = motor::kObsL;
// Measured on the NineBot S 2026-07-27 (I/f spin, 'flux' column): 0.025-0.027.
static constexpr float kObsLambda = motor::kObsLambda;
static constexpr float kObsFluxGain = 1.0e5f;
static constexpr float kObsPllKp = 300.0f;
static constexpr float kObsPllKi = 22500.0f;

// CSA at GAIN_20 (not the BSP's GAIN_5 constant): 10 A is 200 mV, ~0.021 A/LSB.
static constexpr float kMvToA = 1.0f / (0.001f * 20.0f * 1000.0f); // 0.05 A/mV

// GPIO2 is ADC1_CH1 — same SAR unit and pin row as the current-sense inputs
// (GPIO4/5/6). Toggling it in the sampling ISR injected charge into the SAR
// 1-2 us before every first-slot aperture and was THE dominant conversion
// corruptor (1322:0 slot asymmetry, 66% pair disagreement at idle). Keep NC;
// if ISR timing ever needs scoping again, use a non-ADC GPIO.
static constexpr auto kScopePin = GPIO_NUM_NC;
static constexpr std::array<adc_channel_t, 3> kPhaseChannels = {
    ADC_CHANNEL_3, // phase A, GPIO4
    ADC_CHANNEL_4, // phase B, GPIO5
    ADC_CHANNEL_5, // phase C, GPIO6
};

namespace {

// Output of the FOC task, read by the 25 Hz stream.
struct FocSnapshot {
  float id, iq;       // measured, rotor frame
  float iqref;        // effective torque-current target
  float vd, vq;       // PI outputs: loop effort. |v|/vlim -> saturation; in a
                      // HOLD soak vd/id is the copper resistance live (winding
                      // temperature proxy); 1.5*(vd*id+vq*iq) is electrical W
  float th_drive_deg; // angle actually driving the SVPWM (synthetic in stages 2-3)
  float rpm_drive;    // mechanical, from the drive frequency
  float th_est_deg;   // observer PLL electrical angle
  float rpm_est;      // observer mechanical speed
  float flux_mag;     // |rotor flux| — should sit near lambda_pm when locked
  char state;         // H hold, S I/f, V converging, C closed (sensorless), X stopping
};

// The FOC task rewrites all five values every control period (up to 20 kHz); the
// stream reads them ~40 ms later. Five independent atomics tear — the stream can
// mix ia from one cycle with ialpha from the next, which is nonsensical since
// ialpha == ia by definition. Publish the whole snapshot atomically with a
// seqlock: writer bumps the sequence odd before the store and even after; the
// reader retries while it saw an odd or changed sequence. Single writer (FOC
// task, CPU1), single reader (stream, CPU0); acquire/release order the payload.
//
// The payload itself is stored word-by-word through relaxed atomics, not as a
// plain struct copy: a concurrent plain copy is a data race the compiler may
// tear or reorder regardless of the fences (Boehm, "Can seqlocks get along
// with programming language memory models?"). With atomic words a torn read
// can still be OBSERVED mid-write, but it is then discarded by the sequence
// check instead of being undefined behavior.
struct FocState {
  static_assert(std::is_trivially_copyable_v<FocSnapshot>);
  static constexpr size_t kSnapWords = (sizeof(FocSnapshot) + 3) / 4;

  std::atomic<uint32_t> seq{0};
  std::array<std::atomic<uint32_t>, kSnapWords> snap_words{};

  std::atomic<uint32_t> task_runs{0};     // FOC-task wakeups
  std::atomic<uint32_t> notifications{0}; // sum of ulTaskNotifyTake() returns
  std::atomic<uint32_t> compute_cyc_max{0};

  void publish(const FocSnapshot &s) {
    uint32_t w[kSnapWords] = {};
    std::memcpy(w, &s, sizeof(s));
    const uint32_t v = seq.load(std::memory_order_relaxed);
    seq.store(v + 1, std::memory_order_relaxed); // odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    for (size_t i = 0; i < kSnapWords; i++)
      snap_words[i].store(w[i], std::memory_order_relaxed);
    seq.store(v + 2, std::memory_order_release); // even: stable
  }
  FocSnapshot read() const {
    uint32_t w[kSnapWords];
    uint32_t before, after;
    do {
      before = seq.load(std::memory_order_acquire);
      for (size_t i = 0; i < kSnapWords; i++)
        w[i] = snap_words[i].load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);
      after = seq.load(std::memory_order_relaxed);
    } while ((before & 1u) || before != after);
    FocSnapshot s;
    std::memcpy(&s, w, sizeof(s));
    return s;
  }
};

// Latest board temperatures, refreshed at 2 Hz by the stream task. File-scope so
// the RTPS telemetry publisher can emit the same values the CSV stream prints.
std::array<float, 4> g_temps = {NAN, NAN, NAN, NAN};

sensorless::FocSampler g_sampler;
sensorless::FluxObserver g_observer;
sensorless::HallPoller g_hall_poller; // interrupt-free: polled each FOC tick
sensorless::HallDrive g_hall_drive;
std::atomic<bool> g_hall_ready{false}; // poller initialized (app_main -> FOC task)
FocState g_foc;
TaskHandle_t g_foc_task = nullptr;
std::shared_ptr<espp::BldcDriver> g_driver;

// Three-phase duty write that lands all three comparators in the SAME PWM
// period. espp's set_pwm() writes the shadow registers one after another, and
// the hardware copies shadow -> active at TEZ: if TEZ falls between the first
// and last write, one phase runs the new duty a whole period before the other
// two. On the NineBot (~670 uH line-line) that one-period mismatch was a 1-2 A
// blip nobody noticed; on a ~4 uH drone outrunner it is 13 A/us into the
// windings and the DRV8353 VDS OCP latches on every boot (phase-A clamp,
// 2026-10-23). The FOC task is the worst offender: it wakes on the TEP sample
// and lands its write right around TEZ. Fix: if the counter is within
// kDutyGuardTicks of TEZ on the way down, wait for TEZ to pass, then write the
// three values back-to-back with interrupts off. Bounded spin (one guard
// window at most, ~3 us); a stopped timer exits through the iteration cap.
static constexpr uint32_t kDutyHalfPeriodTicks = espp::BldcDriver::TICKS_PER_PERIOD / 2; // 2000
static constexpr uint32_t kDutyGuardTicks = 240; // 3 us at 80 MHz
static portMUX_TYPE g_duty_mux = portMUX_INITIALIZER_UNLOCKED;
// defer_past_tez: wait until the counter has passed TEZ (counting up) before
// writing, so the new duties always take effect at the NEXT TEZ, i.e. a fixed
// two periods after the sample that produced them, whatever the compute time
// was (as long as it stays under ~70 us). HFI demodulation needs that fixed
// latency; the plain current loop does not and keeps the shorter path.
static void set_duties(float a, float b, float c, bool defer_past_tez = false) {
  static mcpwm_timer_handle_t timer = nullptr;
  static const sensorless::PwmComparators *cmp = nullptr;
  if (!g_driver)
    return;
  if (!timer) {
    timer = sensorless::borrow_pwm_timer(*g_driver);
    cmp = &sensorless::borrow_pwm_comparators(*g_driver);
  }
  auto ticks = [](float d) {
    d = std::clamp(d, 0.0f, 1.0f);
    return (uint32_t)(d * (float)kDutyHalfPeriodTicks);
  };
  const uint32_t ta = ticks(a), tb = ticks(b), tc = ticks(c);
  // Counter position straight from the peripheral (group 0, timer 0: the BSP
  // creates the one and only MCPWM timer, and the sampler ISR already assumes
  // the same). The public driver has no phase getter in this IDF.
  mcpwm_dev_t *dev = MCPWM_LL_GET_HW(0);
  if (defer_past_tez) {
    for (int i = 0; i < 4000; i++) { // bounded ~30 us: at most one half period of waiting
      if (mcpwm_ll_timer_get_count_direction(dev, 0) == MCPWM_TIMER_DIRECTION_UP)
        break;
    }
  } else {
    for (int i = 0; i < 256; i++) { // bounded: 256 x ~20 ns reads covers the 3 us guard
      const uint32_t cnt = mcpwm_ll_timer_get_count_value(dev, 0);
      const bool down = mcpwm_ll_timer_get_count_direction(dev, 0) == MCPWM_TIMER_DIRECTION_DOWN;
      if (!(down && cnt < kDutyGuardTicks))
        break;
    }
  }
  (void)timer;
  taskENTER_CRITICAL(&g_duty_mux);
  mcpwm_comparator_set_compare_value((*cmp)[0], ta);
  mcpwm_comparator_set_compare_value((*cmp)[1], tb);
  mcpwm_comparator_set_compare_value((*cmp)[2], tc);
  taskEXIT_CRITICAL(&g_duty_mux);
}
std::shared_ptr<espp::PaceRacerBoard::Encoder> g_encoder; // MT6701 SSI, read by BSP task

// Observer parameters, console-tunable. Written by the console, read by the FOC
// task each tick via g_observer.set_params().
std::atomic<float> g_obs_r{kObsR};
std::atomic<float> g_obs_l{kObsL};
std::atomic<float> g_obs_lambda{kObsLambda};
std::atomic<float> g_obs_flux_gain{kObsFluxGain};
std::atomic<float> g_obs_pll_kp{kObsPllKp};
std::atomic<float> g_obs_pll_ki{kObsPllKi};

// Control setpoints, written by the console, read by the FOC task.
std::atomic<float> g_id_target{0.0f};
std::atomic<float> g_iq_target{0.0f};
std::atomic<float> g_theta{0.0f}; // synthetic electrical angle (rad); fixed in stage 2a
std::atomic<float> g_kp{kDefaultKp};
std::atomic<float> g_ki{kDefaultKi};
std::atomic<bool> g_reset_pi{true};
std::atomic<bool> g_reset_obs{true};
// Phase C's current sense reads large spurious spikes during rotation (30-50 A
// at ~0 real current — confirmed by hand-spinning with nothing commanded; A and
// B stay clean). Those bursts defeat the median filter and poison the dq loop.
// So do NOT auto-select: always measure A and B and reconstruct C. This is what
// the board's third shunt is for — pick the two good channels. Re-enable
// auto-select with 'p auto' only if C's sensing is fixed.
std::atomic<bool> g_auto_phase{false};

// Control mode. HOLD: fixed angle + fixed Id/Iq from the console (stage 2a).
// SPIN: I/f startup, then stage-4 handoff to the observer angle (sub-state in
// the FOC task: I/f -> BLEND -> CLOSED, shown as S/B/C in the stream).
// STOPPING: loop stays closed at 0 A targets in a still-rotating frame until
// the current is unloaded, then the stream task applies hardware Hi-Z coast.
// HALL: hall-commutated FOC (this app's reason to exist) — absolute angle from
// the calibrated hall table, works from 0 rpm under load. Sub-modes L/W/Q.
enum Mode { HOLD = 0, SPIN = 1, STOPPING = 2, HALL = 3, RLV = 4 };
std::atomic<int> g_mode{HOLD};

// Hall-drive sub-mode and parameters ('hrun', 'sine', 'hiq').
enum HallSub { HALL_SPEED = 0, HALL_SINE = 1, HALL_TORQUE = 2, HALL_POS = 3 };
std::atomic<int> g_hall_sub{HALL_SPEED};
std::atomic<float> g_hall_amps{0.0f};  // iq clamp = torque ceiling in HALL
// HALL_TORQUE speed governor (mech rpm, 0 = off). Torque in the direction of
// motion is scaled by (vmax - |rpm|) / band and goes negative above vmax, so a
// torque command can never run the wheel past the velocity limit; below
// vmax - band it is untouched. band = 20% of vmax, at least 5 rpm: wider than
// the hall speed noise at chair speeds, so the roll-off is a slope, not a bang.
std::atomic<float> g_hall_vmax{0.0f};
std::atomic<float> g_hall_rpm{0.0f};   // hrun setpoint (mech rpm)
std::atomic<float> g_hall_ramp{50.0f}; // setpoint ramp, rpm/s
std::atomic<float> g_hall_set_pub{0.0f};
std::atomic<bool> g_spd_gains_custom{false}; // 'sg' was used: hv must not overwrite the tune // the ramped speed reference the loop is tracking, for the stream

// Console-driven setpoints ('hv'): a browser or script nudging the speed
// target while the drive runs. Fixed ramp, a demo ceiling well under the
// profile's, and a heartbeat watchdog ('wd <ms>'): if no console line arrives
// within the window while the drive is live, it takes the same gentle
// unload-then-coast path as 'stop'. A closed tab must not leave a wheel
// spinning. 0 = off (the default: a human at a terminal has no heartbeat).
static constexpr float kHvMaxRpm = 100.0f;
static constexpr float kHvRampRpmS = 50.0f;
static constexpr float kHvDefaultAmps = 12.0f; // ~7 N.m on the NineBot: more than the bench's 5 N.m brake cap
std::atomic<uint32_t> g_console_wd_ms{0};
std::unique_ptr<demo::DemoLink> g_demo_link;
std::unique_ptr<demo::EthLink> g_eth_link;
std::atomic<int64_t> g_console_last_us{0};

// Encoder commutation (MT6701 through the bench's 2.5:1 gearing). 15 pp over
// a 2.5 encoder-turns-per-rotor-turn gear makes elec = EXACTLY 6 x the encoder
// angle — an integer, so the single-turn absolute count maps to a unique
// electrical angle with no homing. Offset from `ecal` (or `eofs`); validated
// on the dyno 2026-08-19: slope 6.005, lash ~6 elec-deg incl. friction.
static constexpr float kEncGear = motor::kEncGear; // encoder revs per rotor rev (sign = sense)
static constexpr float kEncToElec = (float)kPolePairs / kEncGear; // 6.0 on the NineBot, -2.0 on the drone
static constexpr float kEncCntToRad = 6.28318530718f / 16384.0f;
static constexpr float kEncRpmTau = 0.05f; // LPF on encoder-derived rotor rpm
std::atomic<int> g_theta_src{0};           // HALL-mode angle source: 0 = hall table, 1 = encoder
// ATOS feedback source (NVS 'fb'): true = the drive commutates and measures on
// the halls, false = on the encoder. The console spin commands still pick their
// own source; this only decides what the supervisor's hooks ask for.
std::atomic<bool> g_fb_hall{true};
std::atomic<float> g_enc_ofs{0.0f};        // elec rad: th = kEncToElec*enc + ofs
std::atomic<bool> g_enc_cal{false};        // offset known — gates erun/eiq

// Position mode (HALL_POS): P(D) on the encoder accumulator (rotor degrees)
// cascading a speed setpoint into the existing speed PI. Gains host-tuned
// via 'pg' — the autotune campaign sweeps them from outside.
std::atomic<float> g_pos_target{0.0f}; // rotor accumulated deg
std::atomic<float> g_pos_maxrpm{60.0f};
std::atomic<float> g_pos_kp{2.0f};   // rpm per deg
std::atomic<float> g_pos_kd{0.0f};   // rpm per (deg/s)
std::atomic<bool> g_pos_kick{false}; // console pulses on epos: re-seed the D term

// R/L identification (RLV mode): fixed d-axis voltage step at theta 0 with
// the current PI bypassed, currents captured at full tick rate below. R comes
// from the V/I slope across several step levels (kills the deadtime offset),
// L from the exponential rise tau = L/R.
std::atomic<float> g_rl_volts{0.0f};

// High-frequency injection (HFI) for standstill / low-speed angle estimation on
// a motor with no sensors (the drone outrunner, 2026-10-23). Square-wave
// voltage injection at half the control rate: +Vh, -Vh on alternate ticks
// along the d-axis of the injection frame. The current change between
// consecutive samples is demodulated with the injection sign: its d-part is
// Vh*Ts/Ld (an online inductance read), its q-part is proportional to
// (Lq-Ld)/(2 Ld Lq) * sin(2*(theta_rotor - theta_inj)) — zero when the
// injection frame sits on the rotor d-axis, so a PLL that nulls it tracks the
// rotor (up to the 180 deg polarity ambiguity, resolved separately later).
//   hscan <Vh> [Id_bias]   step the injection frame 0..350 deg, 10 deg per
//                          step, average the demodulated d/q responses per
//                          angle into the capture ring ('capdump': angle, yd,
//                          yq). Shows the saliency amplitude and 2-theta
//                          pattern directly, before any loop closes on it.
//   hfi <Vh> [Id_bias] [track]  continuous injection; 'track' closes the PLL
//                          and HOLD drives on the estimated angle.
//   hfi 0                  off.   hfig <kp> <ki> [lag] [sign]  PLL gains.
// Vh is clamped: with L ~3 uH every 0.1 V is 1.7 A of per-tick current step,
// and the sampler's slew filter rejects jumps above 5 A.
static constexpr float kHfiMaxVolts = 0.25f;
// Dead-time compensation ('dtc <V> [band_A]'): each phase loses ~Vdt of average
// voltage in the direction of its current (measured 0.13 V at 24 V / 20 kHz on
// the drone bench, = 3.6 A of offset at 36 mohm). Added per phase before the
// SVPWM zero-sequence shift, sign of the phase current with a linear band
// around zero so measurement noise does not chatter it.
std::atomic<float> g_dt_comp_v{0.0f};
std::atomic<float> g_dt_comp_band{0.3f};
std::atomic<float> g_hfi_volts{0.0f};  // 0 = off
std::atomic<float> g_hfi_bias{0.0f};   // Id bias while injecting (keeps the current off the dead-time zero crossing)
std::atomic<bool> g_hfi_track{false};  // PLL closes, HOLD frame = estimate
std::atomic<float> g_hfi_pll_kp{1000.0f}; // rad/s per A of demodulated q response
std::atomic<float> g_hfi_pll_ki{20000.0f};
std::atomic<int> g_hfi_lag{0};         // demod latency in ticks, 0 = pick the lag with the largest d response
std::atomic<int> g_hfi_lag_used{1};    // what the auto choice settled on (status print)
std::atomic<int> g_hfi_half{2};        // ticks per half-cycle of the injection pattern (2 = 5 kHz)
std::atomic<float> g_hfi_sign{1.0f};   // +1/-1: flips the PLL error sign (Lq<Ld motors)
std::atomic<float> g_hfi_theta{0.0f};  // estimate, rad (published)
std::atomic<float> g_hfi_omega{0.0f};  // estimate, rad/s electrical
std::atomic<float> g_hfi_yd{0.0f};     // filtered demod d response, A per tick
std::atomic<float> g_hfi_yq{0.0f};     // filtered demod q response, A per tick
std::atomic<int> g_hscan_left{0};      // scan: angles left to do (0 = idle)
std::atomic<float> g_hscan_volts{0.0f};
std::atomic<float> g_hscan_bias{0.0f};
static constexpr int kHscanAngles = 36;
// Rotating injection ('hrot <Vh> [ms]'): a voltage vector of magnitude Vh
// stepping 90 deg per tick in the STATIONARY frame (fs/4 = 5 kHz). The current
// answers with a positive-sequence part P ~ Vh/(j w Lavg) and a negative-
// sequence part N ~ P * (dL/L) * exp(j 2 theta_r). arg(N) + arg(P) + 180 deg =
// 2 theta_r with the write latency cancelling, |N|/|P| = (Lq-Ld)/(Lq+Ld). The
// inverter's own asymmetry adds a fixed N_s; pin the rotor at a few known
// angles and the rotor term is the part that turns with 2A.
std::atomic<int> g_hrot_left{0};
std::atomic<float> g_hrot_volts{0.0f};
static constexpr int kHrotSettleTicks = 400;
static constexpr int kHscanSettleTicks = 600;  // 30 ms
static constexpr int kHscanAvgTicks = 2400;    // 120 ms
std::atomic<int> g_rl_left{0};

// Capture ring for step-response figures. The host arms it ('cap'), the FOC
// task fills it, the host reads it back ('capdump'). Mode 0 records
// rotor_pos_deg / rotor_rpm / iq at a decimated rate; mode 1 records
// ialpha / ibeta / valpha every tick (for R/L fits).
static constexpr int kCapMax = 3000; // 36 KB
static float g_cap_buf[kCapMax][3];
std::atomic<int> g_cap_n{0};
std::atomic<int> g_cap_want{0}; // 0 = idle
std::atomic<int> g_cap_every{20};
std::atomic<int> g_cap_mode{0};

std::atomic<bool> g_stream_quiet{false}; // 'sq 1': suppress the CSV stream
std::atomic<float> g_sine_amp{0.0f};     // sine sweep amplitude, mech rpm
std::atomic<float> g_sine_period{8.0f};  // sine period, s
std::atomic<float> g_hall_iq{0.0f};      // hiq torque command
// hcal handshake: console arms N ticks PER DIRECTION, FOC task accumulates
// while I/f drives (both directions required — the load-angle bias cancels in
// the two-direction mean), stream task reports the table (FOC can't print).
std::atomic<uint32_t> g_hcal_ticks{0};
std::atomic<int> g_hcal_result{0}; // 0 idle, 1 ok -> report, -1 failed -> report
// 'hofs' trim (deg), accumulated by the console and applied from the FOC task —
// the hall table is read every control tick on CPU1, so the console must not
// rewrite it directly.
std::atomic<float> g_hofs_pending{0.0f};
// Speed-loop error clamp in HALL mode: one garbage rpm sample (hall edge
// bounce) must produce a bounded nudge, not an iq slam to the rail. Wide
// enough that a full-scale step still reaches the current cap through Kp.
static constexpr float kHallErrClampRpm = 120.0f;
// Handoff parameters ('ho <engage_rpm> <revert_rpm> <decay_A_per_s>').
std::atomic<float> g_ho_engage{kHoEngageRpm};
std::atomic<float> g_ho_revert{kHoRevertRpm};
std::atomic<float> g_ho_decay{kHoDecayAps};
// Speed-loop gains ('sg <kp> <ki>', or a MotorTuning over ATOS).
std::atomic<float> g_spd_kp{kSpeedKp};
std::atomic<float> g_spd_ki{kSpeedKi};
// A MotorTuning asked for `persist`: the stream task writes the live gains to
// NVS on its next tick. The supervisor tick must not block on flash.
std::atomic<bool> g_tune_save_pending{false};
// Escalation limits ('lim', 'vl', 'tl') — see the hard-ceiling constants.
std::atomic<float> g_max_target{kMaxTargetAmps};
std::atomic<float> g_vlim{kVoltageLimit};
std::atomic<float> g_temp_limit{kTempLimitC};
// FOC task -> stream/console handshakes (SPI can't run on the control task).
std::atomic<bool> g_request_coast{false};     // apply DRV8353 Hi-Z coast
std::atomic<bool> g_failsafe_msg{false};      // observer fail-safe fired, report it
std::atomic<bool> g_ceiling_msg{false};       // speed collapsed with iq pinned at the clamp
std::atomic<bool> g_coasting{false};          // outputs are Hi-Z; drive cmds must clear

// ATOS side. The supervisor owns the state machine and drives the atomics above
// through hooks (see app_main); the link carries MotorCommand/MotorState.
axiscfg::AxisConfig g_axis;
std::unique_ptr<atoslink::Link> g_link;
std::unique_ptr<supervisor::Supervisor> g_sup;
std::atomic<float> g_enc_rpm_pub{0.0f};   // filtered encoder rotor rpm, for telemetry

// Shaft position and speed from the selected feedback source. Hall position is
// the ISR-validated step accumulator: one step per electrical sector, so
// 360/(6*pp) mech degrees per step (4 deg on the NineBot, 3 on the hub motor).
// Coarse, but multiturn and live whether or not the bridge is driving.
static constexpr float kHallStepRad = 6.28318530718f / (6.0f * (float)kPolePairs);
inline float shaft_position_rad() {
  if (g_fb_hall.load(std::memory_order_relaxed))
    return (float)g_hall_poller.steps() * kHallStepRad;
  return g_encoder ? g_encoder->get_radians() / kEncGear : 0.0f;
}
/// Speed-loop gains follow the feedback source unless 'sg' overrode them.
inline void apply_feedback_gains() {
  if (g_fb_hall.load(std::memory_order_relaxed)) {
    g_spd_kp.store(kHallSpeedKp);
    g_spd_ki.store(kHallSpeedKi);
  } else {
    g_spd_kp.store(kSpeedKp);
    g_spd_ki.store(kSpeedKi);
  }
}
inline float shaft_rpm() {
  if (g_fb_hall.load(std::memory_order_relaxed))
    return g_hall_drive.rpm_filtered();
  return g_enc_rpm_pub.load(std::memory_order_relaxed);
}
std::atomic<uint16_t> g_drv_status{0};    // last DRV8353 fault poll, raw
std::atomic<bool> g_ev_soft_trip{false};  // guard events for the supervisor, edge-triggered
std::atomic<bool> g_ev_hall_fault{false}; // hall inputs illegal while driving on them
std::atomic<bool> g_ev_stall{false};      // no motion at the torque cap for 0.5 s
// Detector budgets, FOC ticks at 20 kHz.
static constexpr int kHallIllegalTicks = 1000; // 50 ms of 000/111
static constexpr int kStallTicks = 10000;      // 0.5 s at the cap with no motion
static constexpr float kStallMinSetRpm = 10.0f; // below this a stopped rotor is the plan
std::atomic<bool> g_ev_thermal{false};
std::atomic<bool> g_ev_overrun{false};
std::atomic<bool> g_ev_drv_fault{false};
std::atomic<float> g_spin_amps{0.0f};         // Id during align, Iq during ramp
std::atomic<float> g_spin_omega_target{0.0f}; // electrical rad/s
std::atomic<float> g_spin_ramp_rate{0.0f};    // electrical rad/s^2
std::atomic<uint32_t> g_spin_align_ticks{0};  // align duration in control ticks

// Textbook PI with clamped integrator (anti-windup) and clamped output, matching
// the current-control harness. Float, FOC-task context.
struct PI {
  float kp{0}, ki{0}, integ{0}, lim{0};
  float step(float err, float dt) {
    integ = std::clamp(integ + ki * err * dt, -lim, lim);
    return std::clamp(kp * err + integ, -lim, lim);
  }
  void reset() { integ = 0; }
};

// High-priority control task pinned to CPU1. Woken by the sampler ISR once per
// (decimated) PWM period; runs the full dq current loop (the float FOC math the
// ISR cannot do). Stage 2a: regulate Id/Iq at a fixed synthetic angle — no
// rotation. Id holds the rotor at that electrical angle; Iq would make torque.
// IRAM_ATTR: the control loop runs up to 20 kHz and is worth keeping out of the
// flash cache. Unlike the sampling ISR (see foc_sampler.hpp, where IRAM_ATTR
// mis-orders the literal pool at -O2 and fails to link with "dangerous
// relocation"), this is an ordinary task function and places cleanly.
//
// Note this improves how long the loop TAKES, not when it starts. Measured
// 2026-08-04: sampler late% is dominated by ISR entry latency, so neither this
// nor a bigger cache moves it — see main/linker.lf.
IRAM_ATTR void foc_task_fn(void *) {
  PI pi_d{}, pi_q{};
  uint32_t last_cyc = 0;
  bool have_last = false;

  for (;;) {
    // pdTRUE clears the count on read, so `got` is how many samples the ISR
    // signalled since we last ran — 1 when we keep up, >1 when we fell behind.
    uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    if (got == 0) { // sampler idle/disabled — hold the loop reset and idle
      have_last = false;
      pi_d.reset();
      pi_q.reset();
      continue;
    }

    const uint32_t c0 = esp_cpu_get_cycle_count();
    float dt = have_last ? (float)(c0 - last_cyc) / 240.0e6f : 50.0e-6f;
    if (dt <= 0.0f || dt > 0.01f)
      dt = 50.0e-6f; // guard the first tick / long gaps
    last_cyc = c0;
    have_last = true;

    pi_d.kp = pi_q.kp = g_kp.load(std::memory_order_relaxed);
    pi_d.ki = pi_q.ki = g_ki.load(std::memory_order_relaxed);
    pi_d.lim = pi_q.lim = g_vlim.load(std::memory_order_relaxed);
    if (g_reset_pi.exchange(false)) {
      pi_d.reset();
      pi_q.reset();
    }
    if (g_reset_obs.exchange(false))
      g_observer.reset();

    // While the bridge coasts (Hi-Z after any stop/trip), the loop output is
    // disconnected from the plant — integrating the ~50 mA measurement noise
    // here wound both integrators to the -vlim rail within ~8 s (seen
    // 2026-08-06: a parked board streaming vd=vq=-20, |v|/vlim 141%), and a
    // HOLD-mode `id`/`iq` re-engage would apply that railed vector as a
    // current bang. Hold the PIs reset; every re-engage then starts from 0 V.
    // STOPPING is unaffected: it unloads with the loop closed BEFORE the
    // coast request, so the live-PI gotcha (§9) is respected.
    if (g_coasting.load(std::memory_order_relaxed)) {
      pi_d.reset();
      pi_q.reset();
    }

    // Measured phase currents -> Clarke -> Park (rotor frame).
    const int px = g_sampler.phase_x();
    const int py = g_sampler.phase_y();
    float i3[3];
    i3[px] = g_sampler.amps_x();
    i3[py] = g_sampler.amps_y();
    i3[3 - px - py] = -(i3[px] + i3[py]);
    const auto ab = foc::clarke(i3[0], i3[1]);

    // Software overcurrent: respond like 'stop', not like a brake — unload to
    // 0 A with the loop closed (works at any angle: zero-current regulation is
    // just current nulling), then the STOPPING path requests Hi-Z. The stream
    // task prints the trip report from the frozen capture.
    if (g_sampler.take_soft_trip()) {
      g_id_target.store(0);
      g_iq_target.store(0);
      g_mode.store(STOPPING);
    }

    // Decide the electrical angle and the dq current targets from the mode.
    // HOLD: both from the console. SPIN: I/f startup, then the stage-4 handoff
    // sub-states — CONVERGE decays iq so the rotor's load angle walks the
    // frames together (see the constants block), then CLOSED drives on the
    // observer angle (true sensorless, torque mode — speed loop is a later
    // stage). STOPPING: rotate the last frame at the last omega with 0 A
    // targets and an INTACT PI so the current unloads without a bemf
    // transient, then hand off to the stream task for Hi-Z coast.
    enum HoState { IF_DRIVE = 0, CONVERGE = 1, CLOSED = 2 };
    static int prev_mode = HOLD;
    static int ho = IF_DRIVE;
    static float iq_cmd = 0.0f; // decayed torque command (CONVERGE and CLOSED)
    static uint32_t flux_bad = 0, engage_ok = 0, conv_ok = 0, conv_stuck = 0, cooldown = 0;
    static float sp_theta = 0.0f, sp_omega = 0.0f, last_th = 0.0f;
    static uint32_t sp_align_left = 0, stop_ticks = 0;
    static bool stop_requested = false;
    static float h_set = 0.0f, h_integ = 0.0f, h_time = 0.0f; // HALL-mode speed loop
    const int mode = g_mode.load(std::memory_order_relaxed);

    // The duration budgets below (kEngageTicks, kStopUnloadTicks, ...) are
    // written in 20 kHz control ticks. Decimation ('n <N>') divides the control
    // rate, so scale them here — otherwise the documented 100 ms unload becomes
    // N x 100 ms at 'n N'.
    const uint32_t dec = std::max<uint32_t>(1, g_sampler.decimation());
    auto in_ticks = [dec](uint32_t ticks_at_20khz) {
      return std::max<uint32_t>(1, ticks_at_20khz / dec);
    };

    // Hall commutation state advances every tick regardless of mode, so the
    // angle/speed are already live when HALL engages and `hs` always reports.
    if (g_hall_ready.load(std::memory_order_relaxed)) {
      // Console 'hofs' trims land here: this task reads the hall table every
      // tick, so the console queues the trim instead of rewriting the table
      // concurrently.
      const float hofs = g_hofs_pending.exchange(0.0f, std::memory_order_relaxed);
      if (hofs != 0.0f)
        g_hall_drive.add_offset_deg(hofs);
      g_hall_poller.poll(dt);
      g_hall_drive.update(g_hall_poller, dt);
    }
    // Encoder commutation state, also live every tick. count_/rpm are atomics
    // the BSP's 1 kHz SSI task refreshes, so the angle here is up to 1 ms
    // stale: 5.4 elec-deg at 60 rpm (0.4% torque) — bump the BSP update rate
    // before chasing higher speeds on the encoder.
    static float enc_rpm_f = 0.0f; // filtered rotor mech rpm
    float enc_theta_e = 0.0f;
    if (g_encoder) {
      enc_theta_e = kEncToElec * (float)g_encoder->get_count() * kEncCntToRad +
                    g_enc_ofs.load(std::memory_order_relaxed);
      enc_theta_e -= kTwoPi * floorf(enc_theta_e / kTwoPi);
      const float erpm = g_encoder->get_rpm() / kEncGear;
      enc_rpm_f += (erpm - enc_rpm_f) * std::min(1.0f, dt / kEncRpmTau);
      g_enc_rpm_pub.store(enc_rpm_f, std::memory_order_relaxed);
    }
    if (mode == SPIN && prev_mode != SPIN) { // entering spin: reset the ramp
      sp_theta = 0.0f;
      sp_omega = 0.0f;
      sp_align_left = in_ticks(g_spin_align_ticks.load(std::memory_order_relaxed));
      iq_cmd = g_spin_amps.load(std::memory_order_relaxed); // full current from the start
      ho = IF_DRIVE;
      flux_bad = engage_ok = conv_ok = conv_stuck = cooldown = 0;
      pi_d.reset();
      pi_q.reset();
      g_observer.reset();
    }
    if (mode == STOPPING && prev_mode != STOPPING) {
      sp_theta = last_th; // frame continuity with whatever was being applied
      stop_ticks = 0;     // sp_omega carries over (0 if we were in HOLD)
      stop_requested = false;
    }
    if (mode == HALL && prev_mode != HALL) {
      // bumpless: setpoint = actual speed, from whichever sensor will drive
      h_set = g_theta_src.load(std::memory_order_relaxed) ? enc_rpm_f : g_hall_drive.rpm_filtered();
      h_integ = 0.0f;
      h_time = 0.0f;
      pi_d.reset();
      pi_q.reset();
    }
    prev_mode = mode;

    float th, idref, iqref;
    char mstate = 'H';
    if (mode == SPIN) {
      const float amps = g_spin_amps.load(std::memory_order_relaxed);
      if (sp_align_left > 0) { // DC align at angle 0
        sp_align_left--;
        th = 0.0f;
        idref = amps;
        iqref = 0.0f;
        mstate = 'S';
      } else {
        const float th_est = g_observer.theta();
        const float rpm_est = g_observer.omega() / kRpmToOmegaE; // mechanical
        const float lam = g_obs_lambda.load(std::memory_order_relaxed);
        const float flux = g_observer.flux_mag();
        const bool flux_ok = flux > kFluxFailLo * lam && flux < kFluxFailHi * lam;
        const float revert = g_ho_revert.load(std::memory_order_relaxed);
        idref = 0.0f;
        if (cooldown)
          cooldown--;
        static float spd_set = 0.0f, spd_integ = 0.0f; // speed loop state
        if (ho == CLOSED) { // observer drives; synthetic state shadows it for revert
          th = th_est;
          sp_theta = th_est;
          sp_omega = g_observer.omega();
          // Outer speed loop: ramp the rpm setpoint toward the run target at
          // the run's ramp rate, PI to an iq command bounded by ±amps. Seeded
          // at the CONVERGE exit (setpoint = actual rpm, integrator = decayed
          // iq), so the transfer is bumpless.
          const float rpm_t = g_spin_omega_target.load(std::memory_order_relaxed) / kRpmToOmegaE;
          const float step = g_spin_ramp_rate.load(std::memory_order_relaxed) / kRpmToOmegaE * dt;
          spd_set += std::clamp(rpm_t - spd_set, -step, step);
          const float serr = spd_set - rpm_est;
          const float ki = g_spd_ki.load(std::memory_order_relaxed);
          // ki = 0 is "integrator off": hold it at 0, seed included.
          spd_integ = ki > 0 ? std::clamp(spd_integ + ki * serr * dt, -amps, amps) : 0.0f;
          iq_cmd =
              std::clamp(g_spd_kp.load(std::memory_order_relaxed) * serr + spd_integ, -amps, amps);
          iqref = iq_cmd;
          mstate = 'C';
          if (rpm_est < revert) {
            // Speed collapsed. If iq was pinned at the clamp, the LOAD beat
            // the torque ceiling — I/f has the same ceiling and no angle
            // feedback, so reverting just pole-slips against the brake (seen
            // on the dyno as oscillation). Gentle-stop and say so. A revert
            // WITH headroom is the original estimate-loss case: back to I/f.
            if (iq_cmd >= 0.95f * amps || iq_cmd <= -0.95f * amps) {
              g_mode.store(STOPPING);
              g_id_target.store(0);
              g_iq_target.store(0);
              g_ceiling_msg.store(true);
              ho = IF_DRIVE;
            } else {
              ho = IF_DRIVE; // continuity: sp_theta/sp_omega already track the observer
              cooldown = in_ticks(kCooldownTicks);
            }
          }
        } else { // synthetic I/f ramp keeps running under IF_DRIVE and CONVERGE
          // Sign-aware ramp: negative rpm targets spin the frame backward
          // (needed for the bidirectional hcal that cancels the load-angle
          // bias). Rate is always positive; approach the target from 0.
          const float wt = g_spin_omega_target.load(std::memory_order_relaxed);
          const float wstep = g_spin_ramp_rate.load(std::memory_order_relaxed) * dt;
          sp_omega += std::clamp(wt - sp_omega, -wstep, wstep);
          sp_theta += sp_omega * dt;
          sp_theta -= kTwoPi * floorf(sp_theta / kTwoPi);
          th = sp_theta;
          if (ho == IF_DRIVE) {
            // Ramped restore: after an abort/revert leaves iq_cmd low, walk it
            // back to the run amps at the decay rate instead of snapping — the
            // instant floor→amps step on conv-stuck abort was a torque bang
            // that raced (and sometimes beat) the soft trip to the VDS OCP.
            iq_cmd = std::min(iq_cmd + g_ho_decay.load(std::memory_order_relaxed) * dt, amps);
            iqref = iq_cmd;
            mstate = 'S';
            // Engage only on a SUSTAINED healthy estimate (a single noisy tick
            // triggered chatter on hardware), and never during the cooldown.
            const bool ok =
                !cooldown && flux_ok && rpm_est >= g_ho_engage.load(std::memory_order_relaxed);
            engage_ok = ok ? engage_ok + 1 : 0;
            if (engage_ok >= in_ticks(kEngageTicks)) {
              ho = CONVERGE;
              iq_cmd = amps;
              engage_ok = conv_ok = conv_stuck = 0;
            }
          } else { // CONVERGE: decay iq, let the load angle align the frames
            const float floor_a = std::min(kIqFloorAmps, amps);
            iq_cmd = std::max(iq_cmd - g_ho_decay.load(std::memory_order_relaxed) * dt, floor_a);
            iqref = iq_cmd;
            mstate = 'V';
            float e = th_est - sp_theta; // frame disagreement, -> 0 as gamma -> 90 deg
            e -= kTwoPi * floorf(e / kTwoPi + 0.5f);
            constexpr float kConvergeRad = kConvergeDeg * 3.14159265f / 180.0f;
            conv_ok = (e < kConvergeRad && e > -kConvergeRad) ? conv_ok + 1 : 0;
            if (iq_cmd <= floor_a)
              conv_stuck++;
            if (conv_ok >= in_ticks(kConvergeTicks)) { // frames agree: switching is a near-no-op
              ho = CLOSED;
              th = th_est;
              sp_theta = th_est;
              sp_omega = g_observer.omega();
              spd_set = rpm_est;  // bumpless speed-loop entry: zero error,
              spd_integ = iq_cmd; // integrator holds the converged torque
            } else if (rpm_est < revert || conv_stuck >= in_ticks(kConvStuckTicks)) {
              ho = IF_DRIVE; // didn't converge / estimate collapsed: back to stiff I/f
              cooldown = in_ticks(kCooldownTicks);
            }
          }
        }
        // Fail-safe: the observer has (or is gaining) authority and its flux
        // estimate left the physical band — the rotor is lost. Coast, never
        // fight a bad angle with real current.
        if (ho != IF_DRIVE) {
          flux_bad = flux_ok ? 0 : flux_bad + 1;
          if (flux_bad >= in_ticks(kFluxFailTicks)) {
            // Same gentle exit as 'stop': closed-loop unload, then Hi-Z. A
            // hard cut here was itself a violent event (bemf brake bang that
            // could fire the hardware OCP and muddy the diagnosis).
            g_mode.store(STOPPING);
            g_id_target.store(0);
            g_iq_target.store(0);
            g_failsafe_msg.store(true);
            ho = IF_DRIVE;
            flux_bad = 0;
          }
        }
      }
    } else if (mode == HALL) {
      // Sensor-commutated FOC: absolute angle from the hall table (hrun/sine/
      // hiq) or the MT6701 (erun/eiq), torque from the speed loop or a direct
      // iq. Works from 0 rpm under load — where the observer can't. The
      // encoder source has no stall blind spot: the angle stays live between
      // hall edges instead of freezing while the rotor creeps.
      const bool esrc = g_theta_src.load(std::memory_order_relaxed) != 0;
      const float rpm_meas = esrc ? enc_rpm_f : g_hall_drive.rpm_filtered();
      const float amps = g_hall_amps.load(std::memory_order_relaxed);
      th = esrc ? enc_theta_e : g_hall_drive.theta();
      idref = 0.0f;
      const int sub = g_hall_sub.load(std::memory_order_relaxed);
      if (sub == HALL_TORQUE) {
        iqref = std::clamp(g_hall_iq.load(std::memory_order_relaxed), -amps, amps);
        // Track so a later switch to speed control enters bumplessly.
        h_set = rpm_meas;
        h_integ = iqref;
        mstate = 'Q';
      } else {
        float target;
        if (sub == HALL_SINE) {
          h_time += dt;
          const float per = std::max(g_sine_period.load(std::memory_order_relaxed), 1.0f);
          target = g_sine_amp.load(std::memory_order_relaxed) * sinf(kTwoPi * h_time / per);
          h_set = target; // sine IS the profile — no extra rate limit on top
          mstate = 'W';
        } else if (sub == HALL_POS) {
          // Position P(D) on the shaft accumulator (encoder or hall steps) -> rpm setpoint.
          const float pos_deg = shaft_position_rad() * (180.0f / 3.14159265f);
          float perr = g_pos_target.load(std::memory_order_relaxed) - pos_deg;
          // On halls the position is quantized to one step, so an error inside
          // one step is not an error: treat it as zero and let the speed
          // integrator bleed off instead of winding up against the detent.
          if (g_fb_hall.load(std::memory_order_relaxed) &&
              std::fabs(perr) <= kHallStepRad * (180.0f / 3.14159265f)) {
            perr = 0.0f;
            h_integ -= h_integ * std::min(1.0f, dt / 0.5f);
          }
          static float perr_prev = 0.0f;
          if (g_pos_kick.exchange(false, std::memory_order_relaxed))
            perr_prev = perr; // no derivative kick on a fresh setpoint
          const float pdterm = (perr - perr_prev) / dt;
          perr_prev = perr;
          const float maxrpm = g_pos_maxrpm.load(std::memory_order_relaxed);
          h_set = std::clamp(g_pos_kp.load(std::memory_order_relaxed) * perr +
                                 g_pos_kd.load(std::memory_order_relaxed) * pdterm,
                             -maxrpm, maxrpm);
          mstate = 'P';
        } else {
          target = g_hall_rpm.load(std::memory_order_relaxed);
          const float step = g_hall_ramp.load(std::memory_order_relaxed) * dt;
          h_set += std::clamp(target - h_set, -step, step);
          mstate = 'L';
        }
        g_hall_set_pub.store(h_set, std::memory_order_relaxed);
        const float serr = std::clamp(h_set - rpm_meas, -kHallErrClampRpm, kHallErrClampRpm);
        const float ki = g_spd_ki.load(std::memory_order_relaxed);
        // ki = 0 is "integrator off": hold it at 0, the torque-mode seed included.
        h_integ = ki > 0 ? std::clamp(h_integ + ki * serr * dt, -amps, amps) : 0.0f;
        iqref = std::clamp(g_spd_kp.load(std::memory_order_relaxed) * serr + h_integ, -amps, amps);
      }
      // Speed governor (see g_hall_vmax): torque in the direction of motion
      // rolls off to zero across a band and goes negative beyond it, whatever
      // the sub-mode asked for. Symmetric, so reverse runaway is caught too.
      // TORQUE: the band sits BELOW the limit, so a torque command settles
      // under it. Speed/position: the band sits ABOVE the limit, so the loop
      // can reach its (already clamped) setpoint and only a released stall or
      // a wound integrator gets cut. Measured 2026-09-25: a stalled speed loop
      // released at the 10 A cap hit 18.5 rad/s against a 6.3 rad/s limit.
      {
        const float vmax = g_hall_vmax.load(std::memory_order_relaxed);
        if (vmax > 0.0f) {
          const float band = std::max(0.2f * vmax, 5.0f);
          const float edge = sub == HALL_TORQUE ? vmax - band : vmax;
          const float fwd_cap = amps * std::clamp((edge + band - rpm_meas) / band, -1.0f, 1.0f);
          const float rev_cap = -amps * std::clamp((edge + band + rpm_meas) / band, -1.0f, 1.0f);
          iqref = std::clamp(iqref, rev_cap, fwd_cap);
        }
      }
      // Frame continuity for STOPPING (trip/'stop'/thermal all exit through it).
      sp_theta = th;
      sp_omega = esrc ? rpm_meas * kRpmToOmegaE : g_hall_drive.omega_e();

      // Feedback guards, only while commutating on the halls. Both exit like a
      // trip: unload through STOPPING, Hi-Z coast, and the supervisor names it.
      //   HALL:  an illegal input state (000/111) for 50 ms. A legal-but-wrong
      //          state from one bad line is caught by the stall guard instead.
      //   STALL: speed or position sub-mode asking for motion, the speed
      //          integrator past half the cap (pushing hard, and smooth
      //          unlike iq, which chatters 6..10 A at a stalled sector edge),
      //          and no net hall progress over 0.5 s (a hand-held wheel
      //          bounces +/-1 step). A blocked wheel, or halls that stopped
      //          reporting. Torque sub-mode is exempt: pushing on a wall at a
      //          commanded torque is legitimate.
      static int hall_illegal_ticks = 0, stall_ticks = 0, stall_ref_steps = 0;
      if (!esrc) {
        const uint8_t hs = g_hall_poller.raw_state();
        hall_illegal_ticks = (hs == 0 || hs == 7) ? hall_illegal_ticks + 1 : 0;
        const int steps_now = g_hall_poller.steps();
        const bool wants_motion = sub != HALL_TORQUE && std::fabs(h_set) > kStallMinSetRpm;
        const bool pushing = amps > 0.0f && std::fabs(h_integ) >= 0.5f * amps;
        bool stalled = false;
        if (wants_motion && pushing) {
          if (stall_ticks == 0)
            stall_ref_steps = steps_now;
          if (++stall_ticks >= kStallTicks) {
            stalled = std::abs(steps_now - stall_ref_steps) <= 2;
            stall_ticks = 0; // next window
          }
        } else {
          stall_ticks = 0;
        }
        if (hall_illegal_ticks >= kHallIllegalTicks || stalled) {
          if (hall_illegal_ticks >= kHallIllegalTicks)
            g_ev_hall_fault.store(true);
          else
            g_ev_stall.store(true);
          hall_illegal_ticks = stall_ticks = 0;
          g_hall_iq.store(0.0f);
          g_hall_rpm.store(0.0f);
          g_mode.store(STOPPING); // takes effect next tick, like a trip
        }
      } else {
        hall_illegal_ticks = stall_ticks = 0;
      }
    } else if (mode == RLV) {
      // R/L identification: the branch only counts down; the actual voltage
      // step is injected after the PI (see below) so the loops can't fight it.
      th = 0.0f;
      idref = 0.0f;
      iqref = 0.0f;
      sp_omega = 0.0f;
      mstate = 'R';
      int left = g_rl_left.load(std::memory_order_relaxed);
      if (left > 0) {
        g_rl_left.store(left - 1, std::memory_order_relaxed);
      } else {
        g_id_target.store(0.0f);
        g_iq_target.store(0.0f);
        g_theta.store(0.0f);
        g_mode.store(HOLD);
      }
    } else if (mode == STOPPING) {
      sp_theta += sp_omega * dt; // keep the frame rotating: no dq frame jump
      sp_theta -= kTwoPi * floorf(sp_theta / kTwoPi);
      th = sp_theta;
      idref = 0.0f;
      iqref = 0.0f;
      mstate = 'X';
      if (!stop_requested && ++stop_ticks >= in_ticks(kStopUnloadTicks)) {
        stop_requested = true;
        g_request_coast.store(true);
      }
    } else { // HOLD
      th = g_theta.load(std::memory_order_relaxed);
      th -= kTwoPi * floorf(th / kTwoPi);
      idref = g_id_target.load(std::memory_order_relaxed);
      iqref = g_iq_target.load(std::memory_order_relaxed);
      sp_omega = 0.0f;
    }
    last_th = th;

    // ---- HFI: demodulate this tick's current against the injection history,
    // run the PLL, and (scan / track) steer the HOLD frame. Injection itself
    // is added to the alpha-beta voltage below. ----
    static float hfi_ia_prev = 0.0f, hfi_ib_prev = 0.0f;   // last sample, alpha-beta
    static float hfi_did_prev = 0.0f, hfi_diq_prev = 0.0f; // last delta, injection frame
    static int hfi_hist[6] = {1, 1, 1, 1, 1, 1}; // sign written at tick k-1, k-2, ... k-6
    static float hfi_th_inj = 0.0f;  // frame the injection is applied in
    static float hfi_th_hat = 0.0f, hfi_w_hat = 0.0f;
    static float hfi_yd_f = 0.0f, hfi_yq_f = 0.0f;
    static int scan_idx = 0, scan_tick = 0, scan_n = 0; static float scan_yd = 0.0f, scan_yq = 0.0f;
    static float scan_yd3[3] = {0, 0, 0}, scan_yq3[3] = {0, 0, 0}; static int scan_n3[3] = {0, 0, 0};
    static float bin_d[32] = {}, bin_q[32] = {}; static int bin_n[32] = {}; // synchronous averages by pattern phase
    static int hfi_pat = 0; // position in the +,+,-,- pattern
    float vinj = 0.0f;
    const float hfi_vh = g_hfi_volts.load(std::memory_order_relaxed);
    const bool scanning = g_hscan_left.load(std::memory_order_relaxed) > 0;
    const bool rotating = g_hrot_left.load(std::memory_order_relaxed) > 0;
    const bool hfi_on = (hfi_vh > 0.0f || scanning || rotating) && (mode == HOLD || mode == SPIN || mode == HALL);
    static int rot_k = 0, rot_n = 0; static float rot_pr = 0, rot_pi = 0, rot_nr = 0, rot_ni = 0; static int rot_left_total = 0;
    float vrot_a = 0.0f, vrot_b = 0.0f;
    if (rotating) {
      const int left = g_hrot_left.load(std::memory_order_relaxed);
      if (rot_left_total == 0) { rot_left_total = left; rot_k = 0; rot_n = 0; rot_pr = rot_pi = rot_nr = rot_ni = 0.0f; }
      // demod this tick's current with THIS tick's phasor (constant latency phases cancel in arg(N)+arg(P))
      const int ph = rot_k & 3; // 0,90,180,270 deg
      const float cph = (ph == 0) ? 1.0f : (ph == 2) ? -1.0f : 0.0f;
      const float sph = (ph == 1) ? 1.0f : (ph == 3) ? -1.0f : 0.0f;
      if (rot_left_total - left >= kHrotSettleTicks) {
        // P += i * e^{-j phi};  N += i * e^{+j phi}
        rot_pr += ab.alpha * cph + ab.beta * sph;  rot_pi += ab.beta * cph - ab.alpha * sph;
        rot_nr += ab.alpha * cph - ab.beta * sph;  rot_ni += ab.beta * cph + ab.alpha * sph;
        rot_n++;
      }
      const float vr = g_hrot_volts.load(std::memory_order_relaxed);
      vrot_a = vr * cph; vrot_b = vr * sph;
      rot_k++;
      g_hrot_left.store(left - 1, std::memory_order_relaxed);
      if (left - 1 == 0) {
        const float inv = 1.0f / (float)std::max(rot_n, 1);
        g_cap_buf[0][0] = rot_pr * inv; g_cap_buf[0][1] = rot_pi * inv; g_cap_buf[0][2] = (float)rot_n;
        g_cap_buf[1][0] = rot_nr * inv; g_cap_buf[1][1] = rot_ni * inv; g_cap_buf[1][2] = vr;
        g_cap_n.store(2, std::memory_order_relaxed); g_cap_mode.store(4);
        rot_left_total = 0;
      }
    } else {
      rot_left_total = 0;
    }
    if (hfi_on) {
      // Demodulation: delta of the stationary-frame current, rotated into the
      // injection frame, second-differenced (kills the slowly varying part),
      // correlated with the sign that produced it.
      const float da = ab.alpha - hfi_ia_prev, db = ab.beta - hfi_ib_prev;
      const float ci = espp::fast_cos(hfi_th_inj), si = espp::fast_sin(hfi_th_inj);
      const float did = da * ci + db * si, diq = -da * si + db * ci;
      // Pattern is `half` ticks of +Vh then `half` of -Vh (default 4+4 = 2.5 kHz),
      // not +,-: a sample at the period centre sees half of the previous
      // period's voltage and half of this one's, so an fs/2 alternation is
      // invisible to it (the current triangle crosses its mean at every
      // centre). The write-to-effect latency is also not fixed - the FOC task
      // lands its duty write before TEZ most ticks (effect next period) but
      // after it on a slow tick (effect the period after) - so a tick is only
      // demodulated when the three writes it could be answering to all had the
      // same sign; then delta(k) = s * Vh * Ts / L exactly, whichever latency
      // applied. 'hfig ... lag' shifts the window if needed.
      // With the deferred write (fixed 2-period latency) the voltage in period
      // k is the sign written at tick k-2, so delta(k) = (s(k-3) + s(k-2))/2 *
      // Vh*Ts/L: hist[1] and hist[2] at this point (lag = 2). The pair sum is
      // +/-1 on the two ticks inside each half-cycle and 0 at the transitions.
      // The latency is not perfectly fixed (it moved between 1 and 2 periods
      // with the bias level on the bench), so three demods run in parallel
      // for lags 1..3 and the one with the largest |d response| is used:
      // 'hfig ... lag 0' = auto (default), 1..3 forces one.
      static float yd_f3[3] = {0, 0, 0}, yq_f3[3] = {0, 0, 0};
      float yd3[3], yq3[3]; bool val3[3];
      for (int L = 1; L <= 3; L++) {
        const float cL = 0.5f * (float)(hfi_hist[L - 1] + hfi_hist[L]);
        val3[L - 1] = cL != 0.0f;
        yd3[L - 1] = val3[L - 1] ? did * cL : 0.0f;
        yq3[L - 1] = val3[L - 1] ? diq * cL : 0.0f;
        if (val3[L - 1]) {
          yd_f3[L - 1] += (yd3[L - 1] - yd_f3[L - 1]) * 0.02f;
          yq_f3[L - 1] += (yq3[L - 1] - yq_f3[L - 1]) * 0.02f;
        }
      }
      int lag = std::clamp(g_hfi_lag.load(std::memory_order_relaxed), 0, 3);
      if (lag == 0) { // the inductive response is positive by construction: pick the largest positive one
        lag = 1;
        for (int L = 2; L <= 3; L++)
          if (yd_f3[L - 1] > yd_f3[lag - 1])
            lag = L;
      }
      const bool valid = val3[lag - 1];
      const float yd = yd3[lag - 1], yq = yq3[lag - 1];
      hfi_yd_f = yd_f3[lag - 1]; hfi_yq_f = yq_f3[lag - 1];
      g_hfi_lag_used.store(lag, std::memory_order_relaxed);
      (void)hfi_did_prev; (void)hfi_diq_prev;
      g_hfi_yd.store(hfi_yd_f, std::memory_order_relaxed);
      g_hfi_yq.store(hfi_yq_f, std::memory_order_relaxed);

      if (scanning) {
        // Step the injection frame through 36 angles; HOLD frame = scan frame,
        // Id bias on it; average yd/yq per angle into the capture ring.
        const int left = g_hscan_left.load(std::memory_order_relaxed);
        scan_idx = kHscanAngles - left;
        const float ang = (float)scan_idx * (kTwoPi / (float)kHscanAngles);
        // Only the INJECTION frame scans. The HOLD frame (console 'th') and the
        // Id bias stay put, so a free rotor is pinned by the bias on a known
        // axis instead of following the scan: yq must cross zero where the
        // scan angle meets 'th' (mod 180), the direct saliency check.
        hfi_th_inj = ang;
        if (mode == HOLD) { idref = g_hscan_bias.load(std::memory_order_relaxed); iqref = 0.0f; }
        if (scan_tick >= kHscanSettleTicks) {
          for (int L = 0; L < 3; L++)
            if (val3[L]) { scan_yd3[L] += yd3[L]; scan_yq3[L] += yq3[L]; scan_n3[L]++; }
          if (valid) { scan_yd += yd; scan_yq += yq; scan_n++; }
          // Synchronous average of the injection-frame current by pattern
          // phase: noise averages over hundreds of pattern periods and the
          // write latency only rotates the bins, so the contrast between the
          // two halves of the pattern is read off whichever rotation gives
          // the largest d response.
          const int nb = 2 * std::clamp(g_hfi_half.load(std::memory_order_relaxed), 1, 16);
          const int j = hfi_pat % nb;
          bin_d[j] += ab.alpha * ci + ab.beta * si;
          bin_q[j] += -ab.alpha * si + ab.beta * ci;
          bin_n[j]++;
        }
        if (++scan_tick >= kHscanSettleTicks + kHscanAvgTicks) {
          if (scan_idx < kCapMax) {
            // per-angle: the lag with the largest |d response| wins (same rule
            // as the PLL); row = angle, yd, yq. The forced-lag case uses it.
            int best = std::clamp(g_hfi_lag.load(std::memory_order_relaxed), 0, 3) - 1;
            if (best < 0) {
              best = 0;
              for (int L = 1; L < 3; L++)
                if (scan_yd3[L] / (float)std::max(scan_n3[L], 1) >
                    scan_yd3[best] / (float)std::max(scan_n3[best], 1))
                  best = L;
            }
            (void)best;
            const int nb = 2 * std::clamp(g_hfi_half.load(std::memory_order_relaxed), 1, 16);
            const int hb = nb / 2;
            float md[32], mq[32];
            for (int b = 0; b < nb; b++) {
              md[b] = bin_d[b] / (float)std::max(bin_n[b], 1);
              mq[b] = bin_q[b] / (float)std::max(bin_n[b], 1);
            }
            int jbest = 0; float ad_best = -1e9f, aq_best = 0.0f;
            for (int j0 = 0; j0 < nb; j0++) { // contrast between the half starting at j0 and the other half
              float ad = 0.0f, aq = 0.0f;
              for (int b = 0; b < nb; b++) {
                const float w = (((b - j0 + nb) % nb) < hb) ? 1.0f : -1.0f;
                ad += w * md[b]; aq += w * mq[b];
              }
              ad /= (float)hb; aq /= (float)hb; // amplitude of the current square-ish wave, A
              if (ad > ad_best) { ad_best = ad; aq_best = aq; jbest = j0; }
            }
            g_cap_buf[scan_idx][0] = ang * (180.0f / 3.14159265f) + 1000.0f * (float)(jbest + 1); // pattern rotation coded in thousands
            g_cap_buf[scan_idx][1] = ad_best; // d response amplitude (A)
            g_cap_buf[scan_idx][2] = aq_best; // q response amplitude at the same rotation (A)
            g_cap_n.store(scan_idx + 1, std::memory_order_relaxed);
          }
          scan_tick = 0; scan_n = 0; scan_yd = scan_yq = 0.0f;
          for (int L = 0; L < 3; L++) { scan_yd3[L] = scan_yq3[L] = 0.0f; scan_n3[L] = 0; }
          for (int b = 0; b < 32; b++) { bin_d[b] = bin_q[b] = 0.0f; bin_n[b] = 0; }
          g_hscan_left.store(left - 1, std::memory_order_relaxed);
          if (left - 1 == 0) { // done: injection off, bias off (HOLD frame stays where 'th' put it)
            g_hfi_volts.store(0.0f); g_id_target.store(0.0f);
            g_cap_mode.store(3);
          }
        }
        vinj = g_hscan_volts.load(std::memory_order_relaxed);
      } else {
        // PLL on the q response. Error sign is motor-dependent (which axis has
        // the larger inductance); 'hfig ... sign' flips it.
        const float e = valid ? yq * g_hfi_sign.load(std::memory_order_relaxed) : 0.0f;
        hfi_w_hat += g_hfi_pll_ki.load(std::memory_order_relaxed) * e * dt;
        hfi_w_hat = std::clamp(hfi_w_hat, -motor::kMaxRpm * kRpmToOmegaE * 2.0f, motor::kMaxRpm * kRpmToOmegaE * 2.0f);
        hfi_th_hat += (hfi_w_hat + g_hfi_pll_kp.load(std::memory_order_relaxed) * e) * dt;
        hfi_th_hat -= kTwoPi * floorf(hfi_th_hat / kTwoPi);
        const bool track = g_hfi_track.load(std::memory_order_relaxed);
        if (track) {
          hfi_th_inj = hfi_th_hat;
          if (mode == HOLD) { th = hfi_th_hat; idref += g_hfi_bias.load(std::memory_order_relaxed); }
        } else {
          hfi_th_inj = th; // open loop: inject on whatever frame drives (console th, I/f, halls)
          if (mode == HOLD) idref += g_hfi_bias.load(std::memory_order_relaxed);
          hfi_th_hat = hfi_th_inj; hfi_w_hat = sp_omega; // keep the estimate parked on the drive frame
        }
        vinj = hfi_vh;
      }
      g_hfi_theta.store(hfi_th_hat, std::memory_order_relaxed);
      g_hfi_omega.store(hfi_w_hat, std::memory_order_relaxed);
      last_th = th;
    } else {
      hfi_did_prev = hfi_diq_prev = 0.0f; hfi_yd_f = hfi_yq_f = 0.0f; scan_tick = 0; scan_n = 0; scan_yd = scan_yq = 0.0f;
      hfi_th_hat = th; hfi_w_hat = 0.0f; hfi_th_inj = th;
    }
    hfi_ia_prev = ab.alpha; hfi_ib_prev = ab.beta;
    if (vinj != 0.0f) { // half x (+), half x (-) pattern; remember what was written for the demod
      const int half = std::clamp(g_hfi_half.load(std::memory_order_relaxed), 1, 16);
      const int s_now = (hfi_pat < half) ? 1 : -1;
      hfi_pat = (hfi_pat + 1) % (2 * half);
      vinj *= (float)s_now;
      for (int i = 5; i > 0; i--)
        hfi_hist[i] = hfi_hist[i - 1];
      hfi_hist[0] = s_now;
    }

    // Hall calibration: while I/f drives (SPIN in 'S', align done), the drive
    // angle is ground truth for whatever sector the halls currently report.
    // Requires equal ticks in each spin direction (load-angle cancellation).
    static bool cal_active = false;
    const uint32_t cal_arm_ticks = g_hcal_ticks.load(std::memory_order_relaxed);
    if (cal_arm_ticks > 0 && mode == SPIN && mstate == 'S' && sp_align_left == 0 &&
        g_hall_ready.load(std::memory_order_relaxed)) {
      if (!cal_active) {
        g_hall_drive.cal_arm(in_ticks(cal_arm_ticks));
        cal_active = true;
      }
      // Accumulate ONLY while the rotor actually rotates (a hall transition
      // within the last 50 ms). Pre-breakaway/hand-start stall time otherwise
      // smears a full sweep of drive angles into one parked sector — that
      // poisoned a whole table on the dyno while looking perfectly uniform.
      const int r = g_hall_poller.since_s() < 0.05f
                        ? g_hall_drive.cal_step(g_hall_poller.sector(), th, sp_omega >= 0.0f)
                        : 0;
      if (r != 0) {
        g_hcal_result.store(r, std::memory_order_relaxed);
        g_hcal_ticks.store(0, std::memory_order_relaxed);
        cal_active = false;
      }
    } else if (cal_arm_ticks == 0) {
      cal_active = false; // console can re-arm cleanly
    }

    const float s = espp::fast_sin(th);
    const float c = espp::fast_cos(th);
    const auto dq = foc::park(ab, s, c);

    // Two PI current loops -> dq voltage -> inverse Park -> SVPWM -> duties.
    float vd = pi_d.step(idref - dq.d, dt);
    float vq = pi_q.step(iqref - dq.q, dt);
    if (mode == RLV) { // open-loop voltage step for the R/L fit
      vd = g_rl_volts.load(std::memory_order_relaxed);
      vq = 0.0f;
      pi_d.reset();
      pi_q.reset();
    }
    auto vab = foc::inverse_park({vd, vq}, s, c);
    if (vinj != 0.0f) { // HFI: square-wave on the injection frame's d-axis
      vab.alpha += vinj * espp::fast_cos(hfi_th_inj);
      vab.beta += vinj * espp::fast_sin(hfi_th_inj);
    }
    vab.alpha += vrot_a; // rotating injection (0 when idle)
    vab.beta += vrot_b;
    float dtc[3] = {0.0f, 0.0f, 0.0f};
    const float dtv = g_dt_comp_v.load(std::memory_order_relaxed);
    if (dtv > 0.0f) {
      const float band = std::max(g_dt_comp_band.load(std::memory_order_relaxed), 0.01f);
      for (int p = 0; p < 3; p++)
        dtc[p] = dtv * std::clamp(i3[p] / band, -1.0f, 1.0f);
    }
    const auto duties = foc::svpwm(vab, kBusVoltage, kMinDuty, kMaxDuty, dtv > 0.0f ? dtc : nullptr);
    set_duties(duties.a, duties.b, duties.c, /*defer_past_tez=*/hfi_on);
    if (g_auto_phase.load(std::memory_order_relaxed))
      g_sampler.select_lowest_two(duties.a, duties.b, duties.c);

    // Flux observer — runs in parallel, NO control authority (the I/f angle above
    // still drives). Feed it this tick's measured current and the PREVIOUS tick's
    // applied voltage (that voltage produced this current). Purely for logging
    // theta_est against the halls; nothing downstream uses its output yet.
    static float va_prev = 0.0f, vb_prev = 0.0f;
    g_observer.set_params({g_obs_r.load(std::memory_order_relaxed),
                           g_obs_l.load(std::memory_order_relaxed),
                           g_obs_lambda.load(std::memory_order_relaxed),
                           g_obs_flux_gain.load(std::memory_order_relaxed),
                           g_obs_pll_kp.load(std::memory_order_relaxed),
                           g_obs_pll_ki.load(std::memory_order_relaxed)});
    g_observer.update(ab.alpha, ab.beta, va_prev, vb_prev, dt);
    va_prev = vab.alpha;
    vb_prev = vab.beta;

    constexpr float kRadToDeg = 180.0f / 3.14159265f;
    g_foc.publish({dq.d, dq.q, iqref, vd, vq, th * kRadToDeg, sp_omega / kRpmToOmegaE,
                   (hfi_on ? g_hfi_theta.load(std::memory_order_relaxed) : g_observer.theta()) * kRadToDeg,
                   (hfi_on ? g_hfi_omega.load(std::memory_order_relaxed) : g_observer.omega()) / kRpmToOmegaE,
                   g_observer.flux_mag(), mstate});

    // Fill the capture ring if the host armed it.
    {
      const int want = g_cap_want.load(std::memory_order_relaxed);
      if (want > 0) {
        static int cap_div = 0;
        if (++cap_div >= g_cap_every.load(std::memory_order_relaxed)) {
          cap_div = 0;
          const int n = g_cap_n.load(std::memory_order_relaxed);
          if (n < want && n < kCapMax) {
            const int cm = g_cap_mode.load(std::memory_order_relaxed);
            if (cm == 1) {
              g_cap_buf[n][0] = ab.alpha;
              g_cap_buf[n][1] = ab.beta;
              g_cap_buf[n][2] = vab.alpha;
            } else if (cm == 2) {
              // smooth encoder cal: drive angle vs raw encoder electrical
              // angle (no offset applied), sampled while I/f rotates. The
              // host circular-means (th - enc) over both spin directions.
              float ee = 0.0f;
              if (g_encoder) {
                ee = kEncToElec * (float)g_encoder->get_count() * kEncCntToRad;
                ee -= kTwoPi * floorf(ee / kTwoPi);
              }
              g_cap_buf[n][0] = th * (180.0f / 3.14159265f);
              g_cap_buf[n][1] = ee * (180.0f / 3.14159265f);
              g_cap_buf[n][2] = dq.q;
            } else {
              g_cap_buf[n][0] =
                  g_encoder ? g_encoder->get_radians() / kEncGear * (180.0f / 3.14159265f) : 0.0f;
              g_cap_buf[n][1] = enc_rpm_f;
              g_cap_buf[n][2] = dq.q;
            }
            g_cap_n.store(n + 1, std::memory_order_relaxed);
          } else {
            g_cap_want.store(0, std::memory_order_relaxed);
          }
        }
      }
    }

    const uint32_t dt_cyc = esp_cpu_get_cycle_count() - c0;
    if (dt_cyc > g_foc.compute_cyc_max.load(std::memory_order_relaxed))
      g_foc.compute_cyc_max.store(dt_cyc, std::memory_order_relaxed);
    g_foc.task_runs.fetch_add(1, std::memory_order_relaxed);
    g_foc.notifications.fetch_add(got, std::memory_order_relaxed);
  }
}

void drv_check(const std::shared_ptr<espp::PaceRacerBoard::GateDriver> &gd, const char *where);

// DRV8353 VSENSE overcurrent protection (SEN_OCP), runtime-settable by 'sen'.
// configure_drv honours it so that 'clr' and the boot retry do not re-arm a
// protection the operator has deliberately turned off. SEN_OCP trips on the SP
// pin exceeding SEN_LVL (0.25 V) which across the 1 mOhm shunt is ~250 A, so it
// cannot fire legitimately; VDS OCP and the software trip are the real limits.
// Default OFF: measured 2026-09-23, enabling it trips SA_OC instantly on this
// board with no motor attached and no current flowing, which latches the whole
// bridge off and blocks every operation. Re-enable with 'sen 1' to investigate.
std::atomic<bool> g_sense_ocp{false};

// Write the DRV8353's protection and sense configuration, then read it back.
// Shared by boot, boot's retry loop, and 'clr'. It has to be re-runnable: an
// ENABLE reset pulse longer than the device's reset window puts it through
// sleep, and every register comes back at its power-on default.
bool configure_drv(const std::shared_ptr<espp::PaceRacerBoard::GateDriver> &gd,
                   espp::PaceRacerBoard::GateDriver::OcpControl &ocp,
                   espp::PaceRacerBoard::GateDriver::CsaControl &csa,
                   espp::PaceRacerBoard::GateDriver::DriverControl &dcv, bool trace = false) {
  using GD = espp::PaceRacerBoard::GateDriver;
  std::error_code ec;
  gd->clear_faults(ec);
  if (trace)
    drv_check(gd, "after clr_flt");
  // Force-write DRIVER_CONTROL rather than read-modify-write, so a poisoned
  // COAST bit from an old binary cannot survive.
  GD::DriverControl drv_ctl{.raw = 0x0000};
  // Traced write by write on the first pass: if a specific register write is
  // what asserts the fault, this names it instead of leaving it to inference.
  bool ok = gd->write_driver_control(drv_ctl, ec);
  if (trace)
    drv_check(gd, "w driver_ctl");
  ok = ok && gd->set_ocp_mode(GD::OcpMode::LATCHED_SHUTDOWN, ec) &&
       gd->set_ocp_deglitch(GD::OcpDeglitch::US_4, ec) &&
       gd->set_vds_level(GD::VdsLevel::V_0_06, ec); // ~17-22 A hardware backstop
  if (trace)
    drv_check(gd, "w ocp/vds");
  const bool want_sen = g_sense_ocp.load();
  ok = ok && gd->set_sense_overcurrent_enabled(want_sen, ec) &&
       gd->set_sense_level(GD::SenseLevel::V_0_25, ec) && gd->set_csa_gain(GD::CsaGain::GAIN_20, ec);
  gd->clear_faults(ec); // the CSA write itself can trip SEN_OCP; clear after it
  if (trace)
    drv_check(gd, "w csa");
  gd->clear_faults(ec);
  ocp = gd->read_ocp_control(ec);
  csa = gd->read_csa_control(ec);
  dcv = gd->read_driver_control(ec);
  return ok && !ec && !dcv.coast() && !dcv.brake() &&
         ocp.ocp_mode() == GD::OcpMode::LATCHED_SHUTDOWN &&
         ocp.deglitch() == GD::OcpDeglitch::US_4 && ocp.vds_level() == GD::VdsLevel::V_0_06 &&
         csa.sense_overcurrent_disabled() != want_sen &&
         csa.sense_level() == GD::SenseLevel::V_0_25 && csa.gain() == GD::CsaGain::GAIN_20;
}

// Read and print the DRV8353 fault + VGS status with a label. Boot drives the
// gates in several distinct steps (enable, each set_pwm, zero-cal, set_edge,
// sampler enable) and the periodic poll only looks every 5 s, so without this
// a fault latched anywhere in boot just shows up as "fault at 4.9 s" with no
// idea which step caused it. Two SPI reads, boot-only, no timing cost.
struct DrvCheck {
  const char *where;
  uint16_t f1, vgs;
  bool spi_ok;
};
// Buffered, because boot prints are lost in the USB re-enumeration gap after a
// flash (see the note in the DRV config retry loop). 'bootlog' replays them.
std::array<DrvCheck, 12> g_drv_checks{};
size_t g_drv_check_n = 0;

void drv_check(const std::shared_ptr<espp::PaceRacerBoard::GateDriver> &gd, const char *where) {
  std::error_code ec;
  auto fs = gd->fault_status(ec);
  auto vgs = gd->vgs_status(ec);
  if (g_drv_check_n < g_drv_checks.size())
    g_drv_checks[g_drv_check_n++] = {where, fs.raw, vgs.raw, !ec};
  if (ec) {
    fmt::print("#drvchk {:<16} SPI read failed: {}\n", where, ec.message());
    return;
  }
  fmt::print("#drvchk {:<16} fault1=0x{:04x} vgs2=0x{:04x}{}\n", where, fs.raw, vgs.raw,
             (fs.raw || vgs.raw) ? "  <-- FAULT APPEARED HERE" : "");
}

// Fault-reset pulse on the DRV8353 ENABLE pin. The datasheet's reset window is
// a low pulse of tens of microseconds: long enough to drop the latches, short
// enough not to enter sleep. 50 us sits in that window.
//
// Deliberately NOT BldcDriver::disable()/enable(): those stop and restart the
// MCPWM timer, and the current sampler runs off that timer's ISR, so the
// control loop would go dark and trip its own overrun guard. Driving the pin
// leaves PWM running; the gates go Hi-Z for the pulse, which is what we want.
void drv_reset_pulse() {
  // GPIO_NUM_47 = BSP MOTOR_ENABLE_PIN (protected there, so spelled out, as the
  // 'enc p' handler does for the encoder MISO pin).
  gpio_set_level(GPIO_NUM_47, 0);
  esp_rom_delay_us(50);
  gpio_set_level(GPIO_NUM_47, 1);
  // Datasheet wake time from a reset pulse is ~1 ms; give it margin before SPI.
  std::this_thread::sleep_for(5ms);
}

} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "sensorless", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup — stage 1: synchronized sampler + FOC task (Clarke only, no rotation)");

  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::WARN);
  // The BSP constructor starts LED breathing (a 30 ms timer + two 5 kHz LEDC
  // outputs); kill it — this app measures microsecond-scale analog events.
  bsp.stop_breathing();

  // LM75 board temperatures for the high-power runs: streamed at 2 Hz and
  // backing the 'tl' thermal guard. Missing sensors degrade to NaN columns.
  {
    std::error_code tec;
    if (!bsp.init_temperature_sensors(tec))
      logger.warn("Temperature sensors unavailable ({}) — temps stream as NaN, 'tl' guard idle",
                  tec.message());
  }

  // Skip BldcMotor sensor alignment — it DRIVES THE MOTOR at boot.
  auto motor_cfg = bsp.default_motor_config;
  motor_cfg.zero_electric_offset = 1e-6f;
  motor_cfg.sensor_direction = espp::detail::SensorDirection::CLOCKWISE;
  if (!bsp.init_motor(motor_cfg,
                      {.power_supply_voltage = kBusVoltage, .limit_voltage = kBusVoltage})) {
    logger.error("BSP motor init failed");
    return;
  }
  auto driver = bsp.motor_driver();
  g_driver = driver;
  g_encoder = bsp.encoder();

  // What the gate driver looked like before we wrote anything: separates a
  // fault inherited from power-on from one our own config writes provoke.
  drv_check(bsp.gate_driver(), "virgin pre-cfg");

  // Hardware protection + known-good driver state. An ESP32 reboot does NOT
  // power-cycle the DRV8353, so this also clears whatever was latched before.
  {
    using GD = Bsp::GateDriver;
    GD::OcpControl ocp{};
    GD::CsaControl csa{};
    GD::DriverControl dcv{};
    auto gd = bsp.gate_driver();
    if (!configure_drv(gd, ocp, csa, dcv, /*trace=*/true)) {
      // Do NOT return into a silent console: a one-shot error is lost in the
      // USB re-enumeration gap after reset, and "board totally mute" cost two
      // long debug detours (2026-07-30). Keep shouting; retry so flipping VM
      // on recovers without another reset.
      while (true) {
        fmt::print("! DRV8353 config failed (ocp=0x{:04x} csa=0x{:04x} drv=0x{:04x}) — is VM "
                   "on? Retrying in 2 s\n",
                   ocp.raw, csa.raw, dcv.raw);
        std::this_thread::sleep_for(2s);
        if (configure_drv(gd, ocp, csa, dcv)) {
          fmt::print("#DRV8353 recovered — continuing boot\n");
          break;
        }
      }
    }
    logger.info("DRV8353 armed: CSA GAIN_20, VDS 0.06 V (~17-22 A), SEN 0.25 V, latched");
    drv_check(gd, "after config");
  }

  if (!g_sampler.init({.bsp = &bsp,
                       .driver = driver,
                       .phase_channels = kPhaseChannels,
                       .mv_to_a = kMvToA,
                       .trip_amps = kTripAmps,
                       .scope_pin = kScopePin},
                      logger)) {
    return;
  }

  // Control task before the sampler is notified — it must exist to be notified.
  // CPU1 keeps the 20 kHz control cadence off the console/stream tasks on CPU0.
  // Abort on failure: enabling the sampler and bridge with no task consuming
  // samples would leave drive commands running open-loop.
  if (xTaskCreatePinnedToCore(foc_task_fn, "foc", 4096, nullptr, 20, &g_foc_task, 1) != pdPASS) {
    logger.error("Failed to create the FOC control task — not running");
    return;
  }
  g_sampler.set_notify_task(g_foc_task);

  driver->enable();
  drv_check(bsp.gate_driver(), "after enable");
  set_duties(kCenterDuty, kCenterDuty, kCenterDuty);
  drv_check(bsp.gate_driver(), "after pwm center");

  if (!g_sampler.zero_calibrate(logger)) {
    logger.error("Zero-current reading is noisy or railed — sensing broken, not running");
    return;
  }
  drv_check(bsp.gate_driver(), "after zero cal");
  set_duties(kCenterDuty, kCenterDuty, kCenterDuty);

  if (!g_sampler.set_edge(true /*TEP*/, logger))
    return;
  drv_check(bsp.gate_driver(), "after set_edge");
  set_duties(kCenterDuty, kCenterDuty, kCenterDuty);
  // Enabling the sampler starts the FOC task's control loop. With Id*=Iq*=0 the
  // loop commands 0 V -> centered duties -> no current, so this is safe.
  g_sampler.enable();
  drv_check(bsp.gate_driver(), "after sampler on");
  std::this_thread::sleep_for(200ms);
  drv_check(bsp.gate_driver(), "sampler +200ms");

  // Measure phases A and B, reconstruct C (its sense is noisy under rotation).
  g_sampler.set_phases(0, 1);

  // Hall sensor: ground-truth angle for validating the observer. Never used in
  // the control path — it exists here only to log theta_hall alongside theta_est.
  // Halls: POLLED in the FOC task, never interrupts — a floating/oscillating
  // hall line as an any-edge interrupt source wedged CPU0 on first wire-up
  // (2026-07-30) and killed the console before the banner. See hall_drive.hpp.
  g_hall_poller.init(kHallA, kHallB, kHallC);
  g_hall_drive.init(kRpmToOmegaE);
  g_hall_ready.store(true); // publish AFTER init — the FOC task gates on this
  drv_check(bsp.gate_driver(), "after halls");

  // ---- ATOS: provisioning, supervisor, link -----------------------------------
  //
  // NVS first: the encoder offset and hall table come back so the board boots
  // driveable; the axis id decides whether the link joins any topic at all.
  axiscfg::load(g_axis, logger);
  if (g_axis.enc_calibrated()) {
    g_enc_ofs.store(g_axis.enc_ofs_rad);
    g_enc_cal.store(true);
  }
  if (g_axis.hall_calibrated()) {
    g_hall_drive.set_centers_deg(g_axis.hall_deg.data());
  }
  g_fb_hall.store(g_axis.feedback == axiscfg::Feedback::HALL);
  apply_feedback_gains();
  // A tune saved over ATOS (MotorTuning.persist) or 'tune save' overrides the
  // per-source defaults, and 'hv' must not put them back.
  if (g_axis.tuning_saved()) {
    g_spd_kp.store(std::clamp(g_axis.spd_kp, motor::kTuneKpMinRpm, motor::kTuneKpMaxRpm));
    g_spd_ki.store(std::clamp(g_axis.spd_ki, motor::kTuneKiMinRpm, motor::kTuneKiMaxRpm));
    g_spd_gains_custom.store(true);
  }
  if (!std::isnan(g_axis.pll_hz))
    g_hall_drive.set_pll_hz(g_axis.pll_hz);
  logger.info("axis: {} | feedback {} ({}) | enc ofs {} | hall table {} | motor {} | tune {}",
              axiscfg::axis_name(g_axis.axis_id), axiscfg::feedback_name(g_axis.feedback),
              g_axis.feedback_calibrated() ? "calibrated" : "NOT CALIBRATED, cannot arm",
              g_axis.enc_calibrated() ? "restored" : "UNKNOWN ('ecal')",
              g_axis.hall_calibrated() ? "restored" : "UNKNOWN ('hcal')", motor::kName,
              g_axis.tuning_saved() ? "restored from NVS" : "compiled defaults");

  // The hooks are the only way the supervisor reaches the FOC atomics. Each is
  // idempotent: the supervisor re-applies the active one every 10 ms tick, so a
  // hook is a "make it so", not an event. All run on the supervisor task, which
  // may use SPI (the coast write) like the stream task does.
  supervisor::Hooks hooks;
  hooks.coast = [&bsp]() {
    g_id_target.store(0);
    g_iq_target.store(0);
    g_hall_iq.store(0);
    g_mode.store(HOLD);
    if (!g_coasting.load()) {
      std::error_code cec;
      bsp.gate_driver()->set_coast(true, cec);
      if (cec) {
        g_request_coast.store(true); // the stream task retries until it lands
      } else {
        g_coasting.store(true);
        g_reset_pi.store(true);
      }
    }
  };
  hooks.uncoast = [&bsp]() {
    if (g_coasting.exchange(false)) {
      std::error_code cec;
      bsp.gate_driver()->set_coast(false, cec);
      if (cec) {
        g_coasting.store(true); // still Hi-Z; the next tick tries again
        fmt::print("! uncoast SPI write failed: {}\n", cec.message());
      }
      g_reset_pi.store(true);
    }
  };
  hooks.torque = [](float iq, float cap, float max_rpm) {
    g_hall_amps.store(std::clamp(cap, 0.0f, g_max_target.load()));
    g_hall_vmax.store(std::clamp(max_rpm, 0.0f, motor::kMaxRpm));
    g_hall_iq.store(iq);
    g_hall_sub.store(HALL_TORQUE);
    g_theta_src.store(g_fb_hall.load() ? 0 : 1);
    g_mode.store(HALL);
  };
  hooks.velocity = [](float rpm, float ramp, float cap, float max_rpm) {
    g_hall_amps.store(std::clamp(cap, 0.0f, g_max_target.load()));
    g_hall_vmax.store(std::clamp(max_rpm, 0.0f, motor::kMaxRpm));
    g_hall_rpm.store(std::clamp(rpm, -motor::kMaxRpm, motor::kMaxRpm));
    g_hall_ramp.store(ramp);
    g_hall_sub.store(HALL_SPEED);
    g_theta_src.store(g_fb_hall.load() ? 0 : 1);
    g_mode.store(HALL);
  };
  hooks.position = [](float deg, float maxrpm, float cap, bool fresh) {
    g_pos_target.store(deg);
    g_pos_maxrpm.store(std::clamp(maxrpm, 1.0f, motor::kMaxRpm));
    g_hall_vmax.store(std::clamp(maxrpm, 0.0f, motor::kMaxRpm));
    g_hall_amps.store(std::clamp(cap, 0.0f, g_max_target.load()));
    if (fresh)
      g_pos_kick.store(true);
    g_hall_sub.store(HALL_POS);
    g_theta_src.store(g_fb_hall.load() ? 0 : 1);
    g_mode.store(HALL);
  };
  hooks.idle = []() {
    g_id_target.store(0);
    g_iq_target.store(0);
    g_hall_iq.store(0);
    g_hall_rpm.store(0);
    if (g_mode.load() == HALL)
      g_mode.store(HOLD);
  };
  // Mode changes must NOT reset the current-loop PI while the bridge is live
  // (SENSORLESS_FOC.md s9: a reset at speed is a bemf transient). The speed
  // loop enters bumplessly on its own (HALL_TORQUE tracks h_set/h_integ); the
  // position loop only needs its derivative re-seeded.
  hooks.reset_controllers = []() { g_pos_kick.store(true); };
  hooks.measure = []() {
    supervisor::Measured m;
    const auto sn = g_foc.read();
    m.position_rad = shaft_position_rad();
    m.velocity_rad_s = shaft_rpm() * motor::kRpmToRadS;
    m.iq = sn.iq;
    m.id = sn.id;
    m.vd = sn.vd;
    m.vq = sn.vq;
    m.temps = g_temps;
    m.drv_status = g_drv_status.load();
    m.coasting = g_coasting.load();
    m.sampler_enabled = g_sampler.is_enabled();
    m.fb_calibrated = g_fb_hall.load() ? g_hall_drive.calibrated() : (g_encoder && g_enc_cal.load());
    m.soft_trip = g_ev_soft_trip.exchange(false);
    m.thermal = g_ev_thermal.exchange(false);
    m.overrun = g_ev_overrun.exchange(false);
    m.drv_fault = g_ev_drv_fault.exchange(false);
    m.hall_fault = g_ev_hall_fault.exchange(false);
    m.stall = g_ev_stall.exchange(false);
    m.speed_kp_rpm = g_spd_kp.load();
    m.speed_ki_rpm = g_spd_ki.load();
    m.pll_hz = g_hall_drive.pll_hz();
    return m;
  };
  // Outer-loop tune from the wire, already clamped by the supervisor. Live: the
  // integrator keeps its state, only the slope changes (ki = 0 zeroes it, in the loop). Marked custom so 'hv'
  // does not revert it; the flash write, if asked for, happens on the stream task.
  hooks.tuning = [](float kp_rpm, float ki_rpm, float pll_hz, bool persist) {
    g_spd_gains_custom.store(true);
    g_spd_kp.store(kp_rpm);
    g_spd_ki.store(ki_rpm);
    g_hall_drive.set_pll_hz(pll_hz);
    if (persist)
      g_tune_save_pending.store(true);
  };
  g_sup = std::make_unique<supervisor::Supervisor>(std::move(hooks), g_axis.axis_id);
  if (!g_axis.assigned())
    g_sup->raise(rammp::FaultCode::UNASSIGNED_AXIS);

  // 100 Hz supervisor tick. Priority 6 on CPU0: above the stream/console
  // (default 0..5), well below the FOC task (20, CPU1). Absolute deadline so a
  // slow SPI coast write does not stretch the watchdog resolution.
  auto sup_task = espp::Task({.callback =
                                  [](std::mutex &m, std::condition_variable &cv) -> bool {
                                    static auto next = std::chrono::steady_clock::now();
                                    g_sup->tick();
                                    next += 10ms;
                                    const auto now = std::chrono::steady_clock::now();
                                    if (next <= now) {
                                      next = now;
                                      return false;
                                    }
                                    std::unique_lock<std::mutex> lk(m);
                                    cv.wait_until(lk, next);
                                    return false;
                                  },
                              .task_config = {.name = "supervisor",
                                              .stack_size_bytes = 6 * 1024,
                                              .priority = 6,
                                              .core_id = 0},
                              .log_level = espp::Logger::Verbosity::WARN});
  sup_task.start();

  // DHCP ethernet + RTPS. With the axis unassigned the link brings ethernet up
  // and stops there; 'axis <n>' followed by a reboot joins the topics.
  // Ethernet comes up regardless of the axis: once a board is inside a wheel
  // the service stream on tcp :5555 (EthLink, below) is the only way to
  // provision it, and the coexistence measurements showed an idle W5500 costs
  // the control loop nothing.
  g_link = std::make_unique<atoslink::Link>(atoslink::Config{
      .axis = g_axis.axis_id,
      .on_command = [](const rammp::MotorCommand &c) { g_sup->on_command(c); },
      .on_tuning = [](const rammp::MotorTuning &t) { g_sup->on_tuning(t); },
      .state_fn = []() { return g_sup->state(); },
      .hostname = std::string("pace-racer-") + axiscfg::axis_name(g_axis.axis_id),
  });
  const bool eth_up = g_link->start(bsp, logger);
  if (!eth_up)
    logger.error("atos: link start failed; console only");

  // 10 Hz CSV stream + periodic DRV fault poll, silent while disarmed. Raw
  // angle columns are gone on purpose: at spin speeds the electrical period
  // aliases against the stream rate, so th_* sampled at 10 Hz is noise — derr
  // (est - hall, wrapped) is the only angle-derived number that means anything
  // here, and rpm_est vs rpm_hall carries the observer-tracking story.
  // aerr = wrapped th_est - th_drive (deg): the handoff's convergence signal.
  // ~+90 under light-load I/f, walks to ~0 during 'V' as iq decays, ~0 in 'C'.
  // vd/vq are the current-loop PI outputs (host derives P = 1.5*(vd*id+vq*iq),
  // bus current, |v|/vlim saturation, and the vd/id copper-R proxy from them).
  // t0..t3 are the LM75s, refreshed at 2 Hz, NaN when a read fails.
  // rpm_set is the ramped speed reference while a hall speed loop runs, else 0.
  fmt::print("%t, mode, id, iq, iqref, vd, vq, aerr, rpm_drive, rpm_est, rpm_hall, flux, "
             "t0, t1, t2, t3, rpm_set\n");
  auto stream_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::steady_clock::now();
    static int tick = 0;
    // File-scope so the RTPS publisher emits the same values as the CSV stream —
    // the two transports must carry an identical sample to be comparable.
    auto &temps = g_temps;
    ++tick;

    // A MotorTuning with persist set: write the live gains here, off the
    // supervisor tick. Rare, and a few ms on this task costs nothing.
    if (g_tune_save_pending.exchange(false)) {
      const float kp = g_spd_kp.load(), ki = g_spd_ki.load(), hz = g_hall_drive.pll_hz();
      if (axiscfg::save_tuning(kp, ki, hz, logger)) {
        g_axis.spd_kp = kp;
        g_axis.spd_ki = ki;
        g_axis.pll_hz = hz;
        fmt::print("#tune saved kp={:.4g} ki={:.4g} A/rpm pll={:.1f} Hz (ATOS persist)\n", kp, ki, hz);
      }
    }

    // Board temperatures every 5th tick (2 Hz — LM75 conversion is ~100 ms and
    // the I2C traffic, like the DRV fault poll, briefly delays the sampling ISR;
    // late-rejection covers it). The thermal guard fires the same gentle
    // STOPPING path as 'stop' — never a brake — so an unattended soak ends as a
    // clean unload + coast with the stream still telling the story.
    if (tick % 10 == 0 && bsp.temperature_sensors_initialized()) {
      Bsp::TemperatureErrors terrs;
      auto tr = bsp.board_temperatures_c(terrs);
      int hot = -1;
      const float tl = g_temp_limit.load(std::memory_order_relaxed);
      for (size_t i = 0; i < temps.size(); i++) {
        temps[i] = terrs[i] ? NAN : tr[i];
        if (!terrs[i] && tr[i] > tl)
          hot = (int)i;
      }
      const int md = g_mode.load(std::memory_order_relaxed);
      const bool driving =
          g_sampler.is_enabled() &&
          (md == SPIN || md == HALL || g_id_target.load() != 0.0f || g_iq_target.load() != 0.0f);
      if (hot >= 0 && driving) {
        g_id_target.store(0);
        g_iq_target.store(0);
        g_mode.store(STOPPING);
        g_ev_thermal.store(true);
        fmt::print("! thermal guard: T{}={:.1f}C > {:.0f}C limit — unloading to 0 A, then Hi-Z "
                   "coast ('tl' to raise)\n",
                   hot, temps[hot], tl);
      }
    }

    if (g_sampler.take_tripped()) {
      g_ev_soft_trip.store(true);
      auto ti = g_sampler.trip_info();
      auto st = g_sampler.stats();
      // Report only — the FOC task already switched itself to STOPPING
      // (closed-loop unload -> Hi-Z coast). No brake, no disarm, no frame jump.
      // Everything needed to call glitch-vs-real: which limit fired, filtered
      // vs raw at the trip tick, and the filter counters since arm.
      fmt::print(
          "! soft trip >{:.0f}A on {}{} — unloading to 0 A, then Hi-Z coast "
          "(any drive command re-engages)\n"
          "!  filtered A={:+.1f} B={:+.1f} C={:+.1f}  raw {}={:+.1f}\n"
          "!  since arm: imp={}/{}/{} rej={}/{}/{} esc={}/{}/{} max|A|={:.0f}/{:.0f}/{:.0f}\n",
          g_sampler.trip_amps(), "ABC"[ti.trip_phase], ti.trip_phase == ti.recon ? " (recon)" : "",
          ti.amps[0], ti.amps[1], ti.amps[2], "ABC"[ti.raw_phase], ti.raw_amps, st.imp[0],
          st.imp[1], st.imp[2], st.rej[0], st.rej[1], st.rej[2], st.esc[0], st.esc[1], st.esc[2],
          st.max_amps[0], st.max_amps[1], st.max_amps[2]);
      g_sampler.dump_summary();
    }
    if (g_sampler.take_overran()) {
      // Persistent lateness/overrun disabled the sampler, which stops the
      // control task — the bridge would otherwise hold its last duties
      // indefinitely. Force a defined state: zero targets, center the duties
      // (no differential voltage), and request Hi-Z coast below.
      g_mode.store(HOLD);
      g_id_target.store(0);
      g_iq_target.store(0);
      g_reset_pi.store(true);
      set_duties(kCenterDuty, kCenterDuty, kCenterDuty);
      g_request_coast.store(true);
      g_ev_overrun.store(true);
      fmt::print("! sampler persistently late / over budget — disabled; duties centered, "
                 "coasting. 'e 1' to re-arm once the cause is fixed\n");
    }
    // Coast handshake: the FOC task can't touch SPI, so it requests Hi-Z here
    // (end of a gentle stop, the observer fail-safe, or the late/overrun path).
    if (g_request_coast.exchange(false)) {
      std::error_code cec;
      bsp.gate_driver()->set_coast(true, cec);
      if (cec) {
        // A transient SPI failure must not consume the safety request: requeue
        // it so the next stream tick (100 ms) retries until the coast lands.
        g_request_coast.store(true);
        fmt::print("! coast SPI write failed: {} — outputs still live, retrying\n", cec.message());
      } else {
        g_coasting.store(true);
        g_mode.store(HOLD);
        g_reset_pi.store(true);
        fmt::print("#coast — outputs Hi-Z (any drive command re-engages)\n");
      }
    }
    if (g_failsafe_msg.exchange(false)) {
      fmt::print("! observer fail-safe: flux left the band — coasting, disarmed\n");
    }
    if (g_ceiling_msg.exchange(false)) {
      fmt::print("! torque ceiling exceeded: speed collapsed with iq pinned at the clamp — "
                 "unloading to 0 A, then Hi-Z coast (back the brake off; raise `run` amps "
                 "before more load)\n");
    }
    if (int hc = g_hcal_result.exchange(0)) {
      if (hc > 0) {
        for (int sc = 0; sc < 6; sc++)
          g_axis.hall_deg[sc] = g_hall_drive.center_deg(sc);
        axiscfg::save_hall(g_axis.hall_deg, logger);
        fmt::print("#hcal OK — sector centers (deg, saved):");
        for (int sc = 0; sc < 6; sc++)
          fmt::print(" {}={:.1f}", sc, g_hall_drive.center_deg(sc));
        fmt::print("  (hall modes unlocked; trim with 'hofs <deg>')\n");
      } else {
        fmt::print("! hcal FAILED — not every sector seen enough (counts:");
        for (int sc = 0; sc < 6; sc++)
          fmt::print(" {}", g_hall_drive.cal_count(sc));
        fmt::print("). Longer/steadier I/f spin, check hall wiring ('hs')\n");
      }
    }
    // DRV fault poll every ~5 s: the SPI transaction briefly stalls the sampling
    // ISR on CPU0 (the sampler's late-rejection handles the resulting late
    // samples, but keeping it infrequent means fewer to reject).
    if (tick % 100 == 0) {
      std::error_code fec;
      auto fs = bsp.gate_driver()->fault_status(fec);
      if (!fec)
        g_drv_status.store(fs.raw);
      if (!fec && fs.raw != 0) {
        g_ev_drv_fault.store(true);
        g_sampler.disable();
        g_mode.store(HOLD);
        g_id_target.store(0);
        g_iq_target.store(0);
        set_duties(kCenterDuty, kCenterDuty, kCenterDuty);
        fmt::print("! DRV8353 fault 0x{:04x} — disarmed (power-cycle VM to clear)\n", fs.raw);
      }
    }

    // Console heartbeat watchdog ('wd <ms>'). Same exit as 'stop'; one-shot,
    // so the operator sees exactly one line and the drive stays down until a
    // fresh command (which also re-arms the timestamp).
    if (const uint32_t wd = g_console_wd_ms.load(std::memory_order_relaxed); wd > 0) {
      const int64_t age_us = esp_timer_get_time() - g_console_last_us.load(std::memory_order_relaxed);
      const int md = g_mode.load(std::memory_order_relaxed);
      const bool live = g_sampler.is_enabled() && md != HOLD && md != STOPPING;
      if (live && age_us > (int64_t)wd * 1000) {
        g_console_wd_ms.store(0);
        g_hall_rpm.store(0);
        g_hall_iq.store(0);
        g_id_target.store(0);
        g_iq_target.store(0);
        g_mode.store(STOPPING);
        fmt::print("! watchdog: no console line for {} ms — unloading, then Hi-Z coast\n",
                   (int)(age_us / 1000));
      }
    }

    // Silent while disarmed — an idle bench prints nothing; trips, faults, and
    // command echoes are the only unsolicited output.
    if (g_sampler.is_enabled()) {
      float seconds =
          std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();

      // Hall rpm from the polled commutation estimator (signed; the sign
      // convention is the drive frame's once hcal has run).
      float rpm_hall = g_hall_drive.rpm_filtered();

      auto sn = g_foc.read();
      // Observer-vs-drive angle, folded to [-180, 180) — the CONVERGE signal.
      float aerr = sn.th_est_deg - sn.th_drive_deg;
      aerr -= 360.0f * floorf(aerr / 360.0f + 0.5f);
      // 'sq 1' silences the CSV firehose. The host quiets the stream around
      // serial open/close: data in flight when the macOS CDC endpoint closes
      // is the prime suspect for the console wedge that only a USB replug
      // clears (2026-08-20 — esptool can't even reach the ROM afterwards).
      const bool speed_loop = g_mode.load(std::memory_order_relaxed) == HALL &&
                              g_hall_sub.load(std::memory_order_relaxed) != HALL_TORQUE;
      const float rpm_set = speed_loop ? g_hall_set_pub.load(std::memory_order_relaxed) : 0.0f;
      if (g_demo_link) {
        const std::array<float, 13> sample{rpm_set, rpm_hall, sn.iq, sn.iqref, sn.id, sn.vd,
                                           sn.vq, temps[0], temps[1], temps[2], temps[3],
                                           static_cast<float>(sn.state),
                                           g_enc_rpm_pub.load(std::memory_order_relaxed)};
        g_demo_link->emit(sample);
      }
      if (!g_stream_quiet.load(std::memory_order_relaxed) && tick % 2 == 0)
        fmt::print("{:.2f}, {}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.0f}, {:.0f}, {:.0f}, "
                   "{:.0f}, {:.4f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}\n",
                   seconds, sn.state, sn.id, sn.iq, sn.iqref, sn.vd, sn.vq, aerr, sn.rpm_drive,
                   sn.rpm_est, rpm_hall, sn.flux_mag, temps[0], temps[1], temps[2], temps[3],
                   rpm_set);
    }
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 50ms); // 20 Hz: telemetry every tick, CSV every other
    return false;
  };
  // Native USB link. before_ota: flash writes disable the cache, and the
  // sampling ISR is not IRAM-resident (MCPWM_ISR_IRAM_SAFE=n), so the drive
  // must be unloaded and the sampler off before the first DATA frame lands.
  // Shared by the USB and ethernet OTA services.
  std::function<void()> before_ota = [&]() {
    g_console_wd_ms.store(0);
    g_hall_rpm.store(0);
    g_hall_iq.store(0);
    g_id_target.store(0);
    g_iq_target.store(0);
    if (g_sampler.is_enabled() && g_mode.load() != HOLD) {
      g_mode.store(STOPPING); // gentle unload, then the FOC task requests coast
      std::this_thread::sleep_for(400ms);
    }
    g_sampler.disable();
    g_mode.store(HOLD);
    set_duties(kCenterDuty, kCenterDuty, kCenterDuty);
    g_request_coast.store(true);
    fmt::print("#ota: drive unloaded, sampler off\n");
  };
  g_demo_link = std::make_unique<demo::DemoLink>(demo::DemoLink::Config{
      .product = "PACE RACER",
      .serial = "pace-racer-dyno",
      .pid = 0x0d41,
      .channels = {"rpm_set", "rpm_hall", "iq", "iqref", "id", "vd", "vq", "t0", "t1", "t2", "t3",
                   "mode", "rpm_enc"},
      .before_ota = before_ota,
      // Never self-confirm on uptime alone: a wheel board is reached over
      // ethernet only, so EthLink confirms once the network is really up.
      .self_verify_after_s = 0,
  });
  if (std::error_code usb_ec; !g_demo_link->start(usb_ec))
    logger.error("USB link failed: {}", usb_ec.message());

  // The same service stream over TCP (:5555) plus a UDP "PACE?" responder
  // (:5556): flash, console, reset and rollback for a board whose USB port is
  // inside a wheel. Console lines land in DemoLink's queue, so the loop below
  // serves both transports; stdout is teed to the connected client.
  if (eth_up) {
    g_eth_link = std::make_unique<demo::EthLink>(
        demo::EthLink::Config{
            .product = "PACE RACER",
            .before_ota = before_ota,
            .on_line = [](std::string_view l) { g_demo_link->push_line(l); },
            .info_json =
                []() {
                  return fmt::format(
                      "\"axis\":{},\"axis_name\":\"{}\",\"fb\":\"{}\",\"hall_cal\":{},"
                      "\"enc_cal\":{},\"motor\":\"{}\",\"api\":{}",
                      (int)g_axis.axis_id, axiscfg::axis_name(g_axis.axis_id),
                      axiscfg::feedback_name(g_axis.feedback),
                      g_axis.hall_calibrated() ? "true" : "false",
                      g_axis.enc_calibrated() ? "true" : "false", motor::kName,
                      RAMMP_MOTOR_API_VERSION);
                },
            .ip_fn = []() { return g_link ? g_link->ip() : std::string{}; },
        },
        g_demo_link->ota());
    if (g_eth_link->start())
      g_demo_link->set_log_sink([](std::span<const uint8_t> b) { g_eth_link->log_sink(b); });
    else
      logger.error("ethernet service stream failed to start");
  }

  // Capture module (0x30) on the vendor stream: the capture ring, binary and
  // reliable. 'capdump' prints 3000 rows through stdout, and the stdout mirror
  // on the vendor stream is best-effort (drops when the FIFO is full), so a
  // browser reading a 20 kHz R/L trace needs request/reply instead:
  //   0x01 META            -> 0x81 [n u32][every u32][mode u8][want u32][fs_hz f32]
  //   0x02 READ [off u32][count u16] -> 0x82 [off u32][rows: 3 x f32 each]  (count <= 340)
  g_demo_link->worker().register_module(
      0x30,
      [](const espp::stream_frame::Frame &f) {
        if (f.is_reply())
          return;
        std::vector<uint8_t> p;
        auto put_u32 = [&](uint32_t v) {
          for (int i = 0; i < 4; i++)
            p.push_back((v >> (8 * i)) & 0xff);
        };
        auto put_f32 = [&](float v) {
          uint32_t u;
          memcpy(&u, &v, 4);
          put_u32(u);
        };
        if (f.type == 0x01) {
          put_u32((uint32_t)g_cap_n.load());
          put_u32((uint32_t)g_cap_every.load());
          p.push_back((uint8_t)g_cap_mode.load());
          put_u32((uint32_t)g_cap_want.load());
          put_f32(20000.0f);
          g_demo_link->reply(espp::stream_frame::build_frame(true, 0x30, 0x81, p));
        } else if (f.type == 0x02 && f.payload.size() >= 6) {
          uint32_t off = f.payload[0] | (f.payload[1] << 8) | (f.payload[2] << 16) |
                         ((uint32_t)f.payload[3] << 24);
          uint32_t count = f.payload[4] | (f.payload[5] << 8);
          const uint32_t n = (uint32_t)g_cap_n.load();
          if (off > n)
            off = n;
          count = std::min<uint32_t>({count, 340u, n - off});
          put_u32(off);
          for (uint32_t i = 0; i < count; i++)
            for (int c = 0; c < 3; c++)
              put_f32(g_cap_buf[off + i][c]);
          g_demo_link->reply(espp::stream_frame::build_frame(true, 0x30, 0x82, p));
        }
      },
      {.name = "Capture", .app = "", .description = "Binary readout of the 20 kHz capture ring"});

  auto stream_task = espp::Task({.callback = stream_fn,
                                 .task_config = {.name = "stream", .stack_size_bytes = 8 * 1024},
                                 .log_level = espp::Logger::Verbosity::WARN});
  stream_task.start();

  setvbuf(stdin, nullptr, _IONBF, 0);
  fmt::print(
      "#ready bus={:.1f}V pwm=20kHz. ATOS-DRIVE: MotorCommand in, MotorState out.\n"
      "#  ATOS: axis [<0..3>|clear] | fb [hall|enc] | atos | clr (DRV fault reset) | bootlog\n"
      "#  HALL: hcal (after: ho 9999 45 1 | run 2 30 5) then\n"
      "#        hrun <A> <rpm> [ramp] | sine <A> <rpm_amp> <period_s> | hiq <A>\n"
      "#        hv <rpm> [A] (nudge setpoint, 100 rpm cap) | wd <ms> (console heartbeat, 0 off)\n"
      "#        hofs <deg> | hs | hset <s0..s5 deg> | enc   SPIN: run [<A> <rpm> <ramp_s>] | stop\n"
      "#  ENC:  ecal [<A>] | eofs <deg> | erun <A> <rpm> [ramp] | eiq <A>\n"
      "#        epos <deg> [maxrpm] [A] | pg <kp> [kd] | rl [V] [ms] | cap ... | capdump\n"
      "#        hscan <Vh> [Id] | hrot <Vh> [ms] | hfi [Vh [Id [track]]] | hfig <kp> <ki> [lag] [sign] [half] | dtc <V> [band]\n"
      "#  g <kp> <ki> | sg <kp> <ki> | mp/og ... | f | e 1\n"
      "#  limits: lim <target_A> <trip_A> | vl <volts> | tl <degC> | vds <0-4>\n"
      "#gains kp={:.5g} ki={:.5g} | obs R={:.4g} L={:.4g}uH lam={:.5g} fg={:.4g} "
      "pll={:.4g}/{:.4g}\n"
      "#lim target={:.1f}A trip={:.1f}A | vlim {:.1f} V | tlimit {:.0f} C\n",
      kBusVoltage, g_kp.load(), g_ki.load(), g_obs_r.load(), g_obs_l.load() * 1e6f,
      g_obs_lambda.load(), g_obs_flux_gain.load(), g_obs_pll_kp.load(), g_obs_pll_ki.load(),
      g_max_target.load(), g_sampler.trip_amps(), g_vlim.load(), g_temp_limit.load());

  // USB-Serial-JTAG stdin is non-blocking: getchar() returns EOF when idle.
  char line[128];
  size_t len = 0;
  while (true) {
    int ch = g_demo_link ? g_demo_link->getc() : -1; // CDC terminal or a WebUSB console frame
    if (ch < 0)
      ch = getchar(); // UART0 console (idf.py monitor)
    if (ch < 0) {
      std::this_thread::sleep_for(50ms);
      continue;
    }
    if (ch != '\n' && ch != '\r') {
      if (len < sizeof(line) - 1)
        line[len++] = (char)ch;
      continue;
    }
    if (len == 0)
      continue;
    line[len] = '\0';
    len = 0;
    g_console_last_us.store(esp_timer_get_time(), std::memory_order_relaxed);

    // Any drive command must first release a hardware Hi-Z coast.
    auto uncoast = [&]() {
      if (g_coasting.exchange(false)) {
        std::error_code cec;
        bsp.gate_driver()->set_coast(false, cec);
        if (cec)
          fmt::print("! uncoast SPI write failed: {}\n", cec.message());
        g_reset_pi.store(true);
      }
    };

    float a = 0, b = 0;
    int x = 0, y = 0;

    // One master at a time: while MotorCommands are arriving, the console may
    // look, provision and stop, but not drive. 'stop' and 'f' stay allowed.
    {
      static constexpr const char *kDrives[] = {"run",  "hrun", "sine", "hiq", "erun", "eiq",
                                                "epos", "rl",   "ecal", "id ", "iq ",  "th ",
                                                "hv"};
      bool drives = false;
      for (const char *d : kDrives)
        if (strncmp(line, d, strlen(d)) == 0)
          drives = true;
      if (drives && g_sup && g_sup->stream_live()) {
        fmt::print("! refused: a MotorCommand stream is live (age {} ms); stop the MIB first\n",
                   g_sup->state().cmd_age_ms);
        continue;
      }
    }

    if (strncmp(line, "axis", 4) == 0) {
      // axis            show provisioning
      // axis <0..3>     set this board's wheel (takes effect at the next boot)
      // axis clear      factory: unassigned, no calibration
      if (sscanf(line, "axis %d", &x) == 1) {
        if (!axiscfg::valid_axis((uint8_t)x)) {
          fmt::print("! axis: 0..{} only (", (int)rammp::kAxisCount - 1);
          for (const auto &ax : rammp::kAxes)
            fmt::print(" {}={}", (int)ax.id, ax.segment);
          fmt::print(" )\n");
        } else if (axiscfg::save_axis((rammp::AxisId)x, logger)) {
          fmt::print("#axis {} = {} saved; reboot to join its topics\n", x,
                     rammp::axis((rammp::AxisId)x).segment);
        }
      } else if (strcmp(line, "axis clear") == 0) {
        if (axiscfg::clear(logger))
          fmt::print("#axis cleared: unassigned, calibration forgotten; reboot\n");
      } else {
        fmt::print("#axis {} ({}) | fb {}{} | enc ofs {} | hall table {} | motor {} | api v{} fw "
                   "0x{:08x}\n",
                   (int)g_axis.axis_id, axiscfg::axis_name(g_axis.axis_id),
                   axiscfg::feedback_name(g_axis.feedback),
                   g_axis.feedback_calibrated() ? "" : " (NOT CALIBRATED)",
                   g_axis.enc_calibrated() ? "saved" : "NONE",
                   g_axis.hall_calibrated() ? "saved" : "NONE", motor::kName,
                   RAMMP_MOTOR_API_VERSION, motor::kFwVersion);
      }
    } else if (strncmp(line, "fb", 2) == 0 && (line[2] == ' ' || line[2] == 0)) {
      // fb hall|enc — which sensor the ATOS drive commutates and measures on.
      // Takes effect immediately for the next command and is saved for boot.
      // Arming needs the chosen source calibrated: hcal for halls, ecal for enc.
      axiscfg::Feedback fb = g_axis.feedback;
      if (strcmp(line, "fb hall") == 0)
        fb = axiscfg::Feedback::HALL;
      else if (strcmp(line, "fb enc") == 0)
        fb = axiscfg::Feedback::ENCODER;
      else if (line[2] != 0) {
        fmt::print("! fb hall | fb enc\n");
        continue;
      }
      if (line[2] != 0 && axiscfg::save_feedback(fb, logger))
        g_axis.feedback = fb;
      g_fb_hall.store(g_axis.feedback == axiscfg::Feedback::HALL);
      apply_feedback_gains();
      fmt::print("#fb {} ({}) | pos {:.3f} rad rpm {:.1f} | speed gains kp={} ki={} | hall pll {:.1f} Hz\n",
                 axiscfg::feedback_name(g_axis.feedback),
                 g_axis.feedback_calibrated() ? "calibrated" : "NOT CALIBRATED, cannot arm",
                 shaft_position_rad(), shaft_rpm(), g_spd_kp.load(), g_spd_ki.load(),
                 g_hall_drive.pll_hz());
    } else if (sscanf(line, "hpll %f", &a) == 1) {
      // hpll <hz> — hall speed tracking-loop bandwidth (see HallDrive).
      g_hall_drive.set_pll_hz(a);
      fmt::print("#hall pll {:.1f} Hz\n", g_hall_drive.pll_hz());
    } else if (strncmp(line, "tune", 4) == 0 && (line[4] == ' ' || line[4] == 0)) {
      // tune            the outer-loop tune in force, in both unit systems, and what NVS holds
      // tune save       keep the live values across reboots (what MotorTuning.persist does)
      // tune clear      back to the compiled per-source defaults at the next boot
      const float kp = g_spd_kp.load(), ki = g_spd_ki.load(), hz = g_hall_drive.pll_hz();
      if (strcmp(line, "tune save") == 0) {
        if (axiscfg::save_tuning(kp, ki, hz, logger)) {
          g_axis.spd_kp = kp;
          g_axis.spd_ki = ki;
          g_axis.pll_hz = hz;
        }
      } else if (strcmp(line, "tune clear") == 0) {
        if (axiscfg::clear_tuning(logger)) {
          g_axis.spd_kp = g_axis.spd_ki = g_axis.pll_hz = NAN;
        }
      } else if (line[4] != 0) {
        fmt::print("! tune | tune save | tune clear  (live values: sg <kp> <ki>, hpll <hz>)\n");
        continue;
      }
      const auto st = g_sup->state();
      fmt::print("#tune live kp={:.4g} ki={:.4g} A/rpm pll={:.1f} Hz | wire kp={:.4g} A/(rad/s) "
                 "ki={:.4g} A/rad ack={} | nvs {}{}\n",
                 kp, ki, hz, st.speed_kp, st.speed_ki, st.tuning_ack,
                 g_axis.tuning_saved() ? fmt::format("kp={:.4g} ki={:.4g}", g_axis.spd_kp, g_axis.spd_ki)
                                       : std::string("none"),
                 std::isnan(g_axis.pll_hz) ? "" : fmt::format(" pll={:.1f}", g_axis.pll_hz));
    } else if (strcmp(line, "atos") == 0) {
      // Link and state-machine status in one line.
      const auto st = g_sup->state();
      const auto &c = g_link->counters();
      fmt::print("#atos link={} ip={} rx={} tx={}/{} overrun={} matched cmd={} state={} | "
                 "state={} mode={} fault={} cmd_age={}ms seq={} ack={} | pos={:.3f} rad vel={:.3f} "
                 "rad/s tq={:.2f} Nm lim tq={:.1f} vel={:.1f} acc={:.1f} | tune rx={} ack={} "
                 "kp={:.3g} ki={:.3g} pll={:.1f}\n",
                 g_link->up() ? "up" : "down", g_link->ip(), c.rx.load(), c.tx_ok.load(),
                 c.tx_fail.load(), c.overrun.load(), c.cmd_matched.load(), c.state_matched.load(),
                 (int)st.state, (int)st.mode, (int)st.fault_code, st.cmd_age_ms, st.last_cmd_seq,
                 st.clear_fault_ack, st.position, st.velocity, st.torque_est, st.torque_limit_eff,
                 st.vel_limit_eff, st.accel_limit_eff, c.rx_tune.load(), st.tuning_ack, st.speed_kp,
                 st.speed_ki, st.hall_pll_hz);
    } else if (strcmp(line, "bootlog") == 0) {
      // Replay the boot-time DRV fault checks. The first line with a nonzero
      // fault1/vgs2 names the boot step that latched it.
      fmt::print("#bootlog {} checks\n", (int)g_drv_check_n);
      bool named = false;
      for (size_t i = 0; i < g_drv_check_n; i++) {
        const auto &c = g_drv_checks[i];
        const bool bad = c.f1 || c.vgs;
        fmt::print("#  {:<16} fault1=0x{:04x} vgs2=0x{:04x}{}{}\n", c.where, c.f1, c.vgs,
                   c.spi_ok ? "" : " (SPI FAILED)", (bad && !named) ? "  <-- FIRST FAULT" : "");
        if (bad)
          named = true;
      }
      if (!named)
        fmt::print("#  boot was clean — the fault latches after boot\n");
    } else if (strncmp(line, "sen", 3) == 0) {
      // sen <0|1> — turn the DRV8353 VSENSE overcurrent protection (SEN_OCP)
      // off or on. It trips when the SP pin exceeds the SEN_LVL threshold
      // (0.25 V), which across this board's 1 mOhm shunt is ~250 A, so it can
      // never fire legitimately: a latched SA_OC/SB_OC/SC_OC with the motor
      // stationary is spurious. Turning it off leaves VDS OCP (~17-22 A), the
      // sampler's software trip and the supply current limit all in place.
      int en = 1;
      if (sscanf(line, "sen %d", &en) != 1) {
        fmt::print("! sen <0|1>\n");
      } else {
        std::error_code sec;
        g_sense_ocp.store(en != 0);
        bsp.gate_driver()->set_sense_overcurrent_enabled(en != 0, sec);
        bsp.gate_driver()->clear_faults(sec);
        auto csa2 = bsp.gate_driver()->read_csa_control(sec);
        if (sec)
          fmt::print("! sen: SPI failed: {}\n", sec.message());
        else
          fmt::print("#sen {} (csa=0x{:04x}) — VDS OCP and the software trip still armed\n",
                     csa2.sense_overcurrent_disabled() ? "OFF" : "ON", csa2.raw);
      }
    } else if (strncmp(line, "clr", 3) == 0) {
      // clr — reset the DRV8353's latched faults without a VM power cycle:
      // ENABLE reset pulse, re-apply the config (a pulse that overshoots into
      // sleep resets every register), then report what is still asserted.
      // A fault that comes straight back is live, not a leftover latch.
      using GD = Bsp::GateDriver;
      g_mode.store(HOLD);
      g_id_target.store(0);
      g_iq_target.store(0);
      g_hall_iq.store(0);
      g_reset_pi.store(true);
      set_duties(kCenterDuty, kCenterDuty, kCenterDuty);

      auto gd = bsp.gate_driver();
      GD::OcpControl ocp{};
      GD::CsaControl csa{};
      GD::DriverControl dcv{};
      drv_reset_pulse();
      const bool cfg_ok = configure_drv(gd, ocp, csa, dcv);

      std::error_code fec;
      auto fs = gd->fault_status(fec);
      auto vgs = gd->vgs_status(fec);
      if (fec) {
        fmt::print("! clr: SPI read failed: {}\n", fec.message());
      } else if (fs.raw == 0 && vgs.raw == 0) {
        fmt::print("#clr OK — faults cleared (fault1=0x0000 vgs2=0x0000), config {}\n",
                   cfg_ok ? "verified" : "FAILED");
      } else {
        fmt::print("! clr: fault1=0x{:04x} vgs2=0x{:04x} still asserted after reset pulse "
                   "(config {}) —",
                   fs.raw, vgs.raw, cfg_ok ? "verified" : "FAILED");
        if (fs.gate_driver_fault())
          fmt::print(" GDF");
        if (fs.vds_ocp_fault())
          fmt::print(" VDS_OCP");
        if (fs.undervoltage_lockout())
          fmt::print(" UVLO");
        if (fs.overtemperature_shutdown())
          fmt::print(" OTSD");
        if (fs.any_vds_fault())
          fmt::print(" VDS[{}{}{}{}{}{}]", fs.vds_high_side_a() ? "HA" : "",
                     fs.vds_low_side_a() ? "LA" : "", fs.vds_high_side_b() ? "HB" : "",
                     fs.vds_low_side_b() ? "LB" : "", fs.vds_high_side_c() ? "HC" : "",
                     fs.vds_low_side_c() ? "LC" : "");
        fmt::print(" | vgs:");
        static constexpr const char *kVgsNames[6] = {"LC", "HC", "LB", "HB", "LA", "HA"};
        for (int i = 0; i < 6; i++)
          if (vgs.raw & (1u << i))
            fmt::print(" {}", kVgsNames[i]);
        if (vgs.raw & (1u << 6))
          fmt::print(" GDUV");
        if (vgs.raw & (1u << 7))
          fmt::print(" OTW");
        if (vgs.raw & (1u << 8))
          fmt::print(" SC_OC");
        if (vgs.raw & (1u << 9))
          fmt::print(" SB_OC");
        if (vgs.raw & (1u << 10))
          fmt::print(" SA_OC");
        fmt::print("\n  live fault: do NOT drive. Scope the named gate, or retry with the "
                   "motor disconnected to split board from motor.\n");
      }
    } else if (strncmp(line, "run", 3) == 0) {
      // run [<amps> <rpm> <ramp_s>] — I/f open-loop spin. Defaults are gentle.
      float amps = kSpinAmps, rpm = kSpinRpm, ramp = kSpinRampS;
      sscanf(line, "run %f %f %f", &amps, &rpm, &ramp);
      amps = std::clamp(amps, 0.0f, g_max_target.load());
      if (ramp < 0.1f)
        ramp = 0.1f;
      const float wt = rpm * kRpmToOmegaE;
      g_spin_amps.store(amps);
      g_spin_omega_target.store(wt);
      g_spin_ramp_rate.store(std::fabs(wt) / ramp);           // rate positive; wt carries direction
      g_spin_align_ticks.store((uint32_t)(kAlignMs * 20.0f)); // 50 us ticks
      if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        uncoast();
        g_mode.store(SPIN);
        fmt::print("#run I/f: align {:.0f}ms @ {:.1f}A, then ramp to {:.0f} rpm over {:.1f}s "
                   "(Iq={:.1f}A); handoff to observer above {:.0f} rpm\n",
                   kAlignMs, amps, rpm, ramp, amps, g_ho_engage.load());
      }
    } else if (strncmp(line, "stop", 4) == 0) {
      // Gentle stop: targets to 0 with the PI INTACT in a still-rotating frame
      // (no bemf transient), then the FOC task requests Hi-Z coast ~100 ms in.
      g_id_target.store(0);
      g_iq_target.store(0);
      if (g_sampler.is_enabled() && g_mode.load() != HOLD) {
        g_mode.store(STOPPING);
        fmt::print("#stop — unloading to 0 A, then Hi-Z coast\n");
      } else { // nothing driving: coast immediately
        g_mode.store(HOLD);
        g_request_coast.store(true);
        fmt::print("#stop\n");
      }
    } else if (strncmp(line, "hv", 2) == 0 && (line[2] == ' ' || line[2] == '\0')) {
      // hv <rpm> [<amps>] — nudge the hall speed setpoint. Starts the hall speed
      // loop if it is not already running; otherwise only the target moves, at
      // a fixed ramp, so a stream of small steps from a UI behaves like one
      // smooth command instead of restarting the run each time. Clamped to
      // kHvMaxRpm regardless of the profile.
      float rpm = 0.0f, amps = -1.0f;
      const int n = sscanf(line, "hv %f %f", &rpm, &amps);
      const bool running = g_mode.load() == HALL && g_hall_sub.load() == HALL_SPEED;
      const bool use_enc = g_encoder && g_enc_cal.load(); // erun when possible, hrun otherwise
      if (n < 1) {
        fmt::print("#hv set={:.1f} ref={:.1f} rpm | {} | source {} | ceiling {:.0f} rpm | wd {} ms\n",
                   g_hall_rpm.load(), running ? g_hall_set_pub.load() : 0.0f,
                   running ? "speed loop running" : "not running", use_enc ? "encoder" : "halls",
                   kHvMaxRpm, g_console_wd_ms.load());
      } else if (!use_enc && !g_hall_drive.calibrated()) {
        fmt::print("! hall table uncalibrated — run: ho 9999 45 1 | run 2 30 5 | hcal, "
                   "then retry\n");
      } else if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        rpm = std::clamp(rpm, -kHvMaxRpm, kHvMaxRpm);
        if (amps < 0.0f)
          amps = running ? g_hall_amps.load() : kHvDefaultAmps;
        amps = std::clamp(amps, 0.0f, g_max_target.load());
        g_hall_amps.store(amps);
        g_hall_rpm.store(rpm);
        g_hall_ramp.store(kHvRampRpmS);
        if (!running) {
          // Encoder commutation when the offset is known (erun): the angle stays
          // live between hall edges, which is what makes low speed smooth on the
          // dyno. Speed gains follow the source, as 'fb' would set them.
          if (!g_spd_gains_custom.load()) { // a characterization 'sg' wins over the per-source defaults
            g_spd_kp.store(use_enc ? kSpeedKp : kHallSpeedKp);
            g_spd_ki.store(use_enc ? kSpeedKi : kHallSpeedKi);
          }
          g_hall_sub.store(HALL_SPEED);
          g_theta_src.store(use_enc ? 1 : 0);
          uncoast();
          g_mode.store(HALL);
        }
        fmt::print("#hv {:.1f} rpm @ {:.0f} rpm/s, torque ceiling {:.1f}A{}\n", rpm, kHvRampRpmS,
                   amps, running ? "" : (use_enc ? " (encoder speed loop started)" : " (hall speed loop started)"));
      }
    } else if (strcmp(line, "mi") == 0) {
      // Motor / build constants in one machine-readable line, for a host that
      // characterizes the motor and wants to compare against what is compiled in.
      fmt::print("#mi name={} pp={} gear={:.2f} kt={:.3f} r={:.4f} l_uh={:.1f} lambda={:.4f} "
                 "vbus={:.1f} kp={:.4f} ki={:.2f} vlim={:.1f} maxrpm={:.0f} maxamps={:.1f} "
                 "spd_kp={:.4f} spd_ki={:.4f} enc_cal={} hall_cal={}\n",
                 motor::kName, kPolePairs, kEncGear, motor::kKt, g_obs_r.load(),
                 g_obs_l.load() * 1e6f, g_obs_lambda.load(), kBusVoltage, g_kp.load(),
                 g_ki.load(), g_vlim.load(), motor::kMaxRpm, g_max_target.load(),
                 g_spd_kp.load(), g_spd_ki.load(), g_enc_cal.load() ? 1 : 0,
                 g_hall_drive.calibrated() ? 1 : 0);
    } else if (strcmp(line, "hb") == 0) {
      // Heartbeat: nothing but the timestamp update above. Silent on purpose,
      // so a UI can keep 'wd' fed without flooding its own console view.
    } else if (sscanf(line, "wd %d", &x) == 1) {
      g_console_wd_ms.store(x < 0 ? 0u : (uint32_t)x);
      g_console_last_us.store(esp_timer_get_time());
      fmt::print("#wd {}\n", x > 0 ? fmt::format("{} ms", x) : std::string("off"));
    } else if (strncmp(line, "hrun", 4) == 0 || strncmp(line, "sine", 4) == 0 ||
               strncmp(line, "hiq", 3) == 0) {
      // Hall-drive modes. All gated on a completed hcal — driving on the
      // uncalibrated default table can command torque in the wrong direction.
      if (!g_hall_drive.calibrated()) {
        fmt::print("! hall table uncalibrated — run: ho 9999 45 1 | run 2 30 5 | hcal, "
                   "then retry\n");
      } else if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else if (strncmp(line, "hrun", 4) == 0) {
        // hrun <amps> <rpm> [<ramp_s>] — hall-FOC speed mode (works loaded, from 0).
        float amps = 3.0f, rpm = 30.0f, ramp = 3.0f;
        sscanf(line, "hrun %f %f %f", &amps, &rpm, &ramp);
        amps = std::clamp(amps, 0.0f, g_max_target.load());
        if (ramp < 0.1f)
          ramp = 0.1f;
        g_hall_amps.store(amps);
        g_hall_rpm.store(std::clamp(rpm, -motor::kMaxRpm, motor::kMaxRpm));
        g_hall_ramp.store(std::fabs(rpm) / ramp);
        g_hall_sub.store(HALL_SPEED);
        g_theta_src.store(0);
        uncoast();
        g_mode.store(HALL);
        fmt::print("#hall speed: {:.0f} rpm over {:.1f}s, torque ceiling {:.1f}A\n", rpm, ramp,
                   amps);
      } else if (strncmp(line, "sine", 4) == 0) {
        // sine <amps> <rpm_amp> <period_s> — speed follows rpm_amp*sin(2pi t/T),
        // through zero and reversal. Crank the brake between periods.
        float amps = 3.0f, ampr = 30.0f, per = 8.0f;
        sscanf(line, "sine %f %f %f", &amps, &ampr, &per);
        amps = std::clamp(amps, 0.0f, g_max_target.load());
        g_hall_amps.store(amps);
        g_sine_amp.store(std::clamp(ampr, 0.0f, 400.0f));
        g_sine_period.store(std::clamp(per, 1.0f, 600.0f));
        g_hall_sub.store(HALL_SINE);
        g_theta_src.store(0);
        uncoast();
        g_mode.store(HALL);
        fmt::print("#hall sine: ±{:.0f} rpm, {:.1f}s period, torque ceiling {:.1f}A\n",
                   g_sine_amp.load(), g_sine_period.load(), amps);
      } else {
        // hiq <amps> — direct hall-frame torque (stall/slow torque testing).
        float amps = 0.0f;
        sscanf(line, "hiq %f", &amps);
        amps = std::clamp(amps, -g_max_target.load(), g_max_target.load());
        g_hall_iq.store(amps);
        g_hall_amps.store(g_max_target.load());
        g_hall_vmax.store(0.0f); // console torque: no governor
        g_hall_sub.store(HALL_TORQUE);
        g_theta_src.store(0);
        uncoast();
        g_mode.store(HALL);
        fmt::print("#hall iq* {:.2f} A (absolute angle — holds torque at stall)\n", amps);
      }
    } else if (strncmp(line, "erun", 4) == 0 || strncmp(line, "eiq", 3) == 0) {
      // Encoder-drive modes: the HALL machinery with theta = 6*enc + ofs.
      // Gated on a known offset — driving on ofs=0 is the hall-table-
      // uncalibrated mistake all over again.
      if (!g_encoder || !g_enc_cal.load()) {
        fmt::print("! encoder offset unknown — run 'ecal <A>' (or 'eofs <deg>' to restore "
                   "a saved one)\n");
      } else if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else if (strncmp(line, "erun", 4) == 0) {
        // erun <amps> <rpm> [<ramp_s>] — encoder-FOC speed mode.
        float amps = 3.0f, rpm = 30.0f, ramp = 3.0f;
        sscanf(line, "erun %f %f %f", &amps, &rpm, &ramp);
        amps = std::clamp(amps, 0.0f, g_max_target.load());
        if (ramp < 0.1f)
          ramp = 0.1f;
        g_hall_amps.store(amps);
        g_hall_rpm.store(std::clamp(rpm, -motor::kMaxRpm, motor::kMaxRpm));
        g_hall_ramp.store(std::fabs(rpm) / ramp);
        g_hall_sub.store(HALL_SPEED);
        g_theta_src.store(1);
        uncoast();
        g_mode.store(HALL);
        fmt::print("#enc speed: {:.0f} rpm over {:.1f}s, torque ceiling {:.1f}A\n", rpm, ramp,
                   amps);
      } else {
        // eiq <amps> — direct encoder-frame torque (stall creep test: the
        // angle tracks the rotor between hall edges, so no 60-deg blind spot).
        float amps = 0.0f;
        sscanf(line, "eiq %f", &amps);
        amps = std::clamp(amps, -g_max_target.load(), g_max_target.load());
        g_hall_iq.store(amps);
        g_hall_amps.store(g_max_target.load());
        g_hall_vmax.store(0.0f); // console torque: no governor
        g_hall_sub.store(HALL_TORQUE);
        g_theta_src.store(1);
        uncoast();
        g_mode.store(HALL);
        fmt::print("#enc iq* {:.2f} A\n", amps);
      }
    } else if (strncmp(line, "epos", 4) == 0) {
      // epos <rotor_deg> [maxrpm] [amps] — encoder position mode. Target is
      // absolute on the rotor accumulator (read the current value from 'enc'
      // acc/2.5). Same gates as erun.
      if (!g_encoder || !g_enc_cal.load()) {
        fmt::print("! encoder offset unknown — run 'ecal <A>' (or 'eofs <deg>')\n");
      } else if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        float deg = 0.0f, maxrpm = 60.0f, amps = 8.0f;
        if (sscanf(line, "epos %f %f %f", &deg, &maxrpm, &amps) < 1) {
          fmt::print("! epos <rotor_deg> [maxrpm] [amps]\n");
        } else {
          g_pos_target.store(deg);
          g_pos_maxrpm.store(std::clamp(maxrpm, 1.0f, 400.0f));
          g_hall_amps.store(std::clamp(amps, 0.0f, g_max_target.load()));
          g_pos_kick.store(true);
          g_hall_sub.store(HALL_POS);
          g_theta_src.store(1);
          uncoast();
          g_mode.store(HALL);
          fmt::print("#enc pos* {:.1f} deg (maxrpm {:.0f}, ceiling {:.1f}A, kp={:.3g} kd={:.3g})\n",
                     deg, g_pos_maxrpm.load(), g_hall_amps.load(), g_pos_kp.load(),
                     g_pos_kd.load());
        }
      }
    } else if (strncmp(line, "pg", 2) == 0 && line[2] == ' ') {
      // pg <kp> [<kd>] — position gains, rpm per deg / rpm per (deg/s)
      float kp = g_pos_kp.load(), kd = g_pos_kd.load();
      if (sscanf(line, "pg %f %f", &kp, &kd) >= 1) {
        g_pos_kp.store(kp);
        g_pos_kd.store(kd);
        fmt::print("#pos gains kp={:.4g} rpm/deg kd={:.4g} rpm/(deg/s)\n", kp, kd);
      }
    } else if (strncmp(line, "rl", 2) == 0 && (line[2] == ' ' || line[2] == '\0')) {
      // rl [<volts>] [<ms>] — R/L identification: open-loop d-axis voltage
      // step at theta 0, currents captured every tick. Run several levels and
      // fit R from the V/I slope (the deadtime error drops out) and L from
      // the exponential rise. 2.5 V max = ~7.7 A peak into 0.32 ohm.
      float v = 1.5f;
      int ms = 15;
      sscanf(line, "rl %f %d", &v, &ms);
      v = std::clamp(v, 0.1f, 2.5f);
      ms = std::clamp(ms, 2, 100);
      if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        uncoast();
        g_cap_mode.store(1);
        g_cap_every.store(1);
        g_cap_n.store(0);
        g_cap_want.store(std::min(ms * 20, (int)kCapMax));
        g_rl_volts.store(v);
        g_rl_left.store(ms * 20);
        g_mode.store(RLV);
        fmt::print("#rl step {:.2f} V for {} ms — 'capdump' when done\n", v, ms);
      }
    } else if (strncmp(line, "hscan", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
      float vh = 0.05f, bias = 0.0f;
      sscanf(line, "hscan %f %f", &vh, &bias);
      vh = std::clamp(vh, 0.0f, kHfiMaxVolts);
      bias = std::clamp(bias, -4.0f, 4.0f);
      if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        uncoast();
        g_mode.store(HOLD);
        g_iq_target.store(0.0f);
        g_id_target.store(bias); // on the current 'th' frame, which the scan leaves alone
        g_hfi_track.store(false);
        g_hfi_volts.store(0.0f);
        g_hscan_volts.store(vh);
        g_hscan_bias.store(bias);
        g_cap_want.store(0);
        g_cap_n.store(0);
        g_hscan_left.store(kHscanAngles);
        fmt::print("#hscan {:.3f} V, Id bias {:.2f} A, {} angles x {} ms — 'capdump' when done "
                   "(angle_deg, yd, yq in A/tick)\n",
                   vh, bias, kHscanAngles, (kHscanSettleTicks + kHscanAvgTicks) / 20);
      }
    } else if (strncmp(line, "dtc", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
      float v = g_dt_comp_v.load(), band = g_dt_comp_band.load();
      sscanf(line, "dtc %f %f", &v, &band);
      g_dt_comp_v.store(std::clamp(v, 0.0f, 0.5f));
      g_dt_comp_band.store(std::clamp(band, 0.01f, 5.0f));
      fmt::print("#dead-time comp {:.3f} V, linear band +/-{:.2f} A\n", g_dt_comp_v.load(), g_dt_comp_band.load());
    } else if (strncmp(line, "hrot", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
      float vh = 0.05f;
      int ms = 1000;
      sscanf(line, "hrot %f %d", &vh, &ms);
      vh = std::clamp(vh, 0.0f, kHfiMaxVolts);
      ms = std::clamp(ms, 50, 5000);
      if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        uncoast();
        if (g_mode.load() != HOLD) g_mode.store(HOLD);
        g_cap_want.store(0); g_cap_n.store(0);
        g_hrot_volts.store(vh);
        g_hrot_left.store(ms * 20);
        fmt::print("#hrot {:.3f} V for {} ms — 'capdump' when done: row0 = P (re, im, n), row1 = N (re, im, Vh)\n", vh, ms);
      }
    } else if (strncmp(line, "hfig", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
      float kp = g_hfi_pll_kp.load(), ki = g_hfi_pll_ki.load(), sg = g_hfi_sign.load();
      int lag = g_hfi_lag.load(), half = g_hfi_half.load();
      sscanf(line, "hfig %f %f %d %f %d", &kp, &ki, &lag, &sg, &half);
      g_hfi_pll_kp.store(kp);
      g_hfi_pll_ki.store(ki);
      g_hfi_lag.store(std::clamp(lag, 0, 3));
      g_hfi_sign.store(sg < 0 ? -1.0f : 1.0f);
      g_hfi_half.store(std::clamp(half, 1, 16));
      fmt::print("#hfi pll kp={:.4g} ki={:.4g} lag={} sign={:+.0f} half={} ({:.0f} Hz)\n", kp, ki, g_hfi_lag.load(),
                 g_hfi_sign.load(), g_hfi_half.load(), 20000.0f / (2.0f * g_hfi_half.load()));
    } else if (strncmp(line, "hfi", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
      float vh = -1.0f, bias = g_hfi_bias.load();
      char tr[8] = {0};
      const int nargs = sscanf(line, "hfi %f %f %7s", &vh, &bias, tr);
      if (nargs < 1) {
        fmt::print("#hfi vh={:.3f} V bias={:.2f} A track={} th={:.1f} deg w={:.1f} rpm yd={:.4f} yq={:.4f} A/tick "
                   "lag={} scan_left={}\n",
                   g_hfi_volts.load(), g_hfi_bias.load(), (int)g_hfi_track.load(),
                   g_hfi_theta.load() * 180.0f / 3.14159265f, g_hfi_omega.load() / kRpmToOmegaE,
                   g_hfi_yd.load(), g_hfi_yq.load(), g_hfi_lag_used.load(), g_hscan_left.load());
      } else if (vh <= 0.0f) {
        g_hfi_volts.store(0.0f);
        g_hfi_track.store(false);
        g_hscan_left.store(0);
        fmt::print("#hfi off\n");
      } else if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        uncoast();
        g_hfi_bias.store(std::clamp(bias, -4.0f, 4.0f));
        g_hfi_track.store(tr[0] == 't');
        g_hfi_volts.store(std::clamp(vh, 0.0f, kHfiMaxVolts));
        fmt::print("#hfi on vh={:.3f} V bias={:.2f} A track={}\n", g_hfi_volts.load(), g_hfi_bias.load(),
                   (int)g_hfi_track.load());
      }
    } else if (sscanf(line, "sq %d", &x) == 1) {
      g_stream_quiet.store(x != 0);
      fmt::print("#stream {}\n", x ? "quiet" : "on");
    } else if (strncmp(line, "capdump", 7) == 0) {
      const int n = g_cap_n.load();
      fmt::print("#capdump n={} every={} mode={}\n", n, g_cap_every.load(), g_cap_mode.load());
      for (int idx = 0; idx < n; idx++) {
        fmt::print("@{},{:.4f},{:.4f},{:.4f}\n", idx, g_cap_buf[idx][0], g_cap_buf[idx][1],
                   g_cap_buf[idx][2]);
        if ((idx & 63) == 63)
          vTaskDelay(1); // let the USB CDC buffer drain
      }
      fmt::print("#capend\n");
    } else if (strncmp(line, "cap", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
      // cap [<every>] [<n>] [<mode>] — arm the capture ring. every = FOC ticks
      // per row (20 = 1 kHz), mode 0 = pos/rpm/iq, 1 = ialpha/ibeta/valpha.
      int every = 20, n = kCapMax, cmode = 0;
      sscanf(line, "cap %d %d %d", &every, &n, &cmode);
      g_cap_mode.store(std::clamp(cmode, 0, 2));
      g_cap_every.store(std::clamp(every, 1, 20000));
      g_cap_n.store(0);
      g_cap_want.store(std::clamp(n, 1, (int)kCapMax));
      fmt::print("#cap armed every={} n={} mode={}\n", g_cap_every.load(), g_cap_want.load(),
                 g_cap_mode.load());
    } else if (strncmp(line, "ecal", 4) == 0) {
      // ecal [<A>] — measure the encoder->electrical offset. d-axis current
      // locks the rotor at 4 known drive angles, each approached from both
      // directions so gear lash and friction hysteresis average out; the
      // offset is the circular mean of (drive angle - 6*enc). ~10 s, paced
      // from this console task; exits through the normal gentle stop.
      float amps = 6.0f;
      sscanf(line, "ecal %f", &amps);
      amps = std::clamp(amps, 1.0f, g_max_target.load());
      if (!g_encoder) {
        fmt::print("! no encoder\n");
      } else if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' first\n");
      } else {
        constexpr float kDeg2Rad = 3.14159265f / 180.0f;
        uncoast();
        g_iq_target.store(0.0f);
        g_theta.store(0.0f);
        g_mode.store(HOLD);
        for (float ramp_a = 2.0f; ramp_a < amps + 1.9f; ramp_a += 2.0f) {
          g_id_target.store(std::min(ramp_a, amps));
          vTaskDelay(pdMS_TO_TICKS(150));
        }
        float ss = 0.0f, cs = 0.0f;
        for (int p = 0; p < 4; p++) {
          const float base = 90.0f * (float)p;
          for (int dir = -1; dir <= 1; dir += 2) {
            g_theta.store((base + 25.0f * (float)dir) * kDeg2Rad);
            vTaskDelay(pdMS_TO_TICKS(350));
            g_theta.store(base * kDeg2Rad);
            vTaskDelay(pdMS_TO_TICKS(650));
            const float enc_e = kEncToElec * (float)g_encoder->get_count() * kEncCntToRad;
            ss += sinf(base * kDeg2Rad - enc_e);
            cs += cosf(base * kDeg2Rad - enc_e);
          }
        }
        g_id_target.store(0.0f);
        g_mode.store(STOPPING); // unload, then Hi-Z coast — the standard exit
        float ofs = atan2f(ss, cs);
        if (ofs < 0.0f)
          ofs += kTwoPi;
        g_enc_ofs.store(ofs);
        g_enc_cal.store(true);
        if (axiscfg::save_enc_ofs(ofs, logger))
          g_axis.enc_ofs_rad = ofs;
        // consistency: resultant length of the 8 unit vectors, 1.0 = all agree
        fmt::print("#ecal OK ofs={:.2f} deg (elec = {:.1f} x enc + ofs) consistency={:.3f} — "
                   "erun/eiq unlocked\n",
                   ofs / kDeg2Rad, kEncToElec, sqrtf(ss * ss + cs * cs) / 8.0f);
      }
    } else if (sscanf(line, "eofs %f", &a) == 1) {
      float ofs = fmodf(a, 360.0f) * 3.14159265f / 180.0f;
      if (ofs < 0.0f)
        ofs += kTwoPi;
      g_enc_ofs.store(ofs);
      g_enc_cal.store(true);
      if (axiscfg::save_enc_ofs(ofs, logger))
        g_axis.enc_ofs_rad = ofs;
      fmt::print("#enc ofs {:.2f} deg — erun/eiq unlocked, saved\n", ofs * 180.0f / 3.14159265f);
    } else if (strncmp(line, "enc p", 5) == 0) {
      // enc pu | enc pd — bias the encoder MISO line (GPIO35) with the weak
      // internal pull, then read `enc` again. A floating DO follows the pull
      // (pu -> cnt 16383, pd -> cnt 0); a driven line ignores it. This is the
      // is-anything-connected test for the SSI link.
      bool up = line[5] == 'u';
      // GPIO_NUM_35 = BSP ENCODER_SPI_MISO_PIN (protected there, so spelled out)
      gpio_set_pull_mode(GPIO_NUM_35, up ? GPIO_PULLUP_ONLY : GPIO_PULLDOWN_ONLY);
      fmt::print("#enc miso(GPIO35) pull={} — read 'enc' and compare cnt\n", up ? "UP" : "DOWN");
    } else if (strcmp(line, "enc") == 0) {
      // MT6701 snapshot next to the halls. Read-only — the encoder has no
      // control authority yet; the BSP task samples it continuously. The SSI
      // status bits are the mount check: field must be NORMAL, track must be
      // OK, and cnt must move smoothly with the wheel (16384 counts/mech rev).
      if (!g_encoder) {
        fmt::print("! encoder not initialized\n");
      } else {
        auto fs = g_encoder->get_magnetic_field_strength();
        const char *fss =
            fs == espp::PaceRacerBoard::Encoder::MagneticFieldStrength::NORMAL       ? "NORMAL"
            : fs == espp::PaceRacerBoard::Encoder::MagneticFieldStrength::TOO_STRONG ? "TOO_STRONG"
                                                                                     : "TOO_WEAK";
        bool lost =
            g_encoder->get_tracking_status() == espp::PaceRacerBoard::Encoder::TrackingStatus::LOST;
        float the = kEncToElec * (float)g_encoder->get_count() * kEncCntToRad + g_enc_ofs.load();
        the -= kTwoPi * floorf(the / kTwoPi);
        fmt::print("#enc cnt={} mech={:.2f}deg acc={:.1f}deg rpm={:.2f} field={} track={} btn={} "
                   "| elec={:.1f}deg ofs={:.1f}deg cal={:d} src={} | hall sec={} steps={} "
                   "rpm={:.1f}\n",
                   g_encoder->get_count(), g_encoder->get_mechanical_degrees(),
                   g_encoder->get_degrees(), g_encoder->get_rpm(), fss, lost ? "LOST" : "OK",
                   g_encoder->get_push_button() ? 1 : 0, the * 180.0f / 3.14159265f,
                   g_enc_ofs.load() * 180.0f / 3.14159265f, g_enc_cal.load() ? 1 : 0,
                   g_theta_src.load() ? "ENC" : "HALL", g_hall_poller.sector(),
                   g_hall_poller.steps(), g_hall_drive.rpm_filtered());
      }
    } else if (strncmp(line, "hset", 4) == 0) {
      // hset <s0..s5 deg> — restore a hall table captured earlier from `hs`
      // (sector centers, drive-frame degrees). Unlocks hall modes without the
      // hcal spin; intended for re-arming after a VM power-cycle reboot.
      float hc[6];
      if (sscanf(line, "hset %f %f %f %f %f %f", &hc[0], &hc[1], &hc[2], &hc[3], &hc[4], &hc[5]) ==
          6) {
        g_hall_drive.set_centers_deg(hc);
        std::copy(hc, hc + 6, g_axis.hall_deg.begin());
        axiscfg::save_hall(g_axis.hall_deg, logger);
        fmt::print("#hset OK — sector centers (deg):");
        for (int sc = 0; sc < 6; sc++)
          fmt::print(" {}={:.1f}", sc, g_hall_drive.center_deg(sc));
        fmt::print("\n  (hall modes unlocked; trim with 'hofs <deg>')\n");
      } else {
        fmt::print("! hset needs 6 sector centers in degrees (capture them with 'hs')\n");
      }
    } else if (strncmp(line, "hcal", 4) == 0) {
      // hcal [<secs>] — calibrate the hall table against the I/f drive angle,
      // <secs> PER DIRECTION. Spin both ways: ho 9999 45 1, then run 3 30 5 /
      // run 3 -30 5. Completes only when both direction budgets are filled.
      float secs = 6.0f;
      sscanf(line, "hcal %f", &secs);
      secs = std::clamp(secs, 2.0f, 30.0f);
      g_hcal_ticks.store((uint32_t)(secs * 20000.0f));
      fmt::print("#hcal armed: {:.0f}s PER DIRECTION of ROTATING I/f ticks (stall time doesn't "
                 "count — hand-start freely). Spin forward AND reverse (run 3 30 5, stop, "
                 "run 3 -30 5); result prints when both fill ('hs' shows calleft)\n",
                 secs);
    } else if (sscanf(line, "hofs %f", &a) == 1) {
      g_hofs_pending.fetch_add(a, std::memory_order_relaxed);
      fmt::print("#hall offset trim {:+.1f} deg queued (applied within one control tick)\n", a);
    } else if (strcmp(line, "hs") == 0) {
      // A=bit2 B=bit1 C=bit0. Stuck 0b000/0b111 = dead line or no power;
      // a healthy standstill shows one of the six legal states.
      fmt::print("#hall state=0b{:03b} (A={} B={} C={}) sector={} steps={} rpm={:.1f} cal={:d} "
                 "| glitch={} illegal={} bounce={} rej={} interval={:.1f}ms calleft={}/{}\n"
                 "#hall centers(deg):",
                 g_hall_poller.raw_state(), (g_hall_poller.raw_state() >> 2) & 1,
                 (g_hall_poller.raw_state() >> 1) & 1, g_hall_poller.raw_state() & 1,
                 g_hall_poller.sector(), g_hall_poller.steps(), g_hall_drive.rpm_filtered(),
                 g_hall_drive.calibrated() ? 1 : 0, g_hall_poller.glitches(),
                 g_hall_poller.illegals(), g_hall_poller.bounces(), g_hall_poller.rejects(),
                 g_hall_poller.interval_s() * 1e3f, g_hall_drive.cal_fwd_left(),
                 g_hall_drive.cal_rev_left());
      for (int sc = 0; sc < 6; sc++)
        fmt::print(" {}={:.1f}", sc, g_hall_drive.center_deg(sc));
      fmt::print("\n");
    } else if (sscanf(line, "id %f", &a) == 1) {
      // Reject outright with the sampler down: no control updates can run, so
      // uncoasting and accepting the target would leave the bridge live with
      // an unregulated command.
      if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' to enable before commanding current\n");
      } else {
        uncoast();
        g_mode.store(HOLD);
        g_id_target.store(std::clamp(a, -g_max_target.load(), g_max_target.load()));
        fmt::print("#id* {:.3f} A\n", g_id_target.load());
      }
    } else if (sscanf(line, "iq %f", &a) == 1) {
      if (!g_sampler.is_enabled()) {
        fmt::print("! sampler disabled — 'e 1' to enable before commanding current\n");
      } else {
        uncoast();
        g_mode.store(HOLD);
        g_iq_target.store(std::clamp(a, -g_max_target.load(), g_max_target.load()));
        fmt::print("#iq* {:.3f} A\n", g_iq_target.load());
      }
    } else if (sscanf(line, "th %f", &a) == 1) {
      uncoast();
      g_mode.store(HOLD);
      g_theta.store(a * 3.14159265f / 180.0f);
      fmt::print("#theta {:.1f} deg\n", a);
    } else if (strncmp(line, "ho", 2) == 0 && (line[2] == ' ' || line[2] == '\0')) {
      // ho <engage_rpm> <revert_rpm> <decay_A_per_s> — stage-4 handoff tuning.
      float en = g_ho_engage.load(), rv = g_ho_revert.load(), dc = g_ho_decay.load();
      sscanf(line, "ho %f %f %f", &en, &rv, &dc);
      g_ho_engage.store(en);
      g_ho_revert.store(rv);
      g_ho_decay.store(std::clamp(dc, 0.1f, 10.0f));
      fmt::print("#handoff engage={:.0f}rpm revert={:.0f}rpm decay={:.2g}A/s\n", en, rv,
                 g_ho_decay.load());
    } else if (sscanf(line, "sg %f %f", &a, &b) == 2) {
      g_spd_gains_custom.store(true);
      g_spd_kp.store(a);
      g_spd_ki.store(b);
      fmt::print("#speed gains kp={:.4g} A/rpm ki={:.4g} A/rpm-s\n", a, b);
    } else if (sscanf(line, "lim %f %f", &a, &b) == 2) {
      // lim <target_A> <trip_A> — escalation step. Keep the layer order: the
      // trip needs >=2 A of visibility above the target (speed-loop overshoot
      // rides on top of the clamp), and the plausibility gate follows the trip
      // at +3 inside the sampler.
      a = std::clamp(a, 0.5f, kHardMaxTargetAmps);
      b = std::clamp(b, a + 2.0f, kHardMaxTripAmps);
      g_max_target.store(a);
      g_sampler.set_trip_amps(b);
      fmt::print(
          "#lim target={:.1f}A trip={:.1f}A plausible={:.1f}A (hard ceilings {:.0f}/{:.0f})\n", a,
          b, b + 3.0f, kHardMaxTargetAmps, kHardMaxTripAmps);
    } else if (sscanf(line, "vl %f", &a) == 1) {
      // vl <volts> — current-loop voltage clamp. Under load vq ~ iq*R + w*lam:
      // at 20 A the iR term alone is ~6.5 V, so the 8 V boot default saturates
      // the speed loop at low rpm — raise deliberately. Slew-gate check: even
      // at 24 V, di/dt through 2L is ~1.8 A/tick, well under the 5 A gate.
      a = std::clamp(a, 1.0f, kHardMaxVolts);
      g_vlim.store(a);
      fmt::print("#vlim {:.1f} V (hard ceiling {:.0f})\n", a, kHardMaxVolts);
    } else if (sscanf(line, "tl %f", &a) == 1) {
      // tl <degC> — board thermal guard threshold (any LM75, while driving).
      a = std::clamp(a, 30.0f, 120.0f);
      g_temp_limit.store(a);
      fmt::print("#tlimit {:.0f} C\n", a);
    } else if (sscanf(line, "vds %d", &x) == 1 && x >= 0 && x <= 6) {
      // vds <0-5> — DRV8353 hardware OCP threshold. 0-4 are the fine steps
      // (0.06-0.10 V); 5 is the chip's next step, a 2x jump to 0.20 V
      // (~58-74 A cold). At 5 the VDS trip sits beyond the current-sense ADC
      // rail, so the 30 A software trip is the outermost real current
      // protection — high-power runs only, keep them short, thermal guard on.
      // Rdson rises ~1.5-2x with junction temp, so the amp threshold DERATES
      // as the FETs heat: late-run OCP trips at lower current are physics.
      // 6 (0.30 V) added 2026-08-27, user-authorized: three real VDS OCP trips
      // (0x0628 x2, 0x060a) ended 850-950 W attempts at 400 rpm; level 6 is the
      // last-resort rung after the 520 rpm speed route.
      const int lvl = x; // enum indices 0-5 map 1:1 for these steps
      std::error_code vec;
      auto gd = bsp.gate_driver();
      gd->set_vds_level(static_cast<Bsp::GateDriver::VdsLevel>(lvl), vec);
      auto ocp = gd->read_ocp_control(vec);
      if (vec || (int)ocp.vds_level() != lvl) {
        fmt::print("! vds write failed (ocp=0x{:04x}{}{})\n", ocp.raw, vec ? ", " : "",
                   vec ? vec.message() : "");
      } else {
        static constexpr const char *kVdsEstAmps[7] = {"17-22", "20-26", "23-30",
                                                       "26-33", "29-37", "58-74", "87-111"};
        fmt::print("#vds {:.2f}V (~{} A cold, derates hot){}\n",
                   x >= 5 ? 0.20f + 0.10f * (float)(x - 5) : 0.06f + 0.01f * (float)x,
                   kVdsEstAmps[x],
                   x >= 5 ? " — SW 30A trip is now the outer current guard" : "");
      }
    } else if (sscanf(line, "odg %d", &x) == 1 && (x == 1 || x == 2 || x == 4 || x == 8)) {
      // odg <1|2|4|8> — VDS OCP deglitch, us. 8 rides through switching-edge
      // and slip-jerk transients that nuisance-trip at 4; the DC threshold is
      // unchanged.
      std::error_code vec;
      auto gd = bsp.gate_driver();
      const int idx = (x == 1) ? 0 : (x == 2) ? 1 : (x == 4) ? 2 : 3;
      gd->set_ocp_deglitch(static_cast<Bsp::GateDriver::OcpDeglitch>(idx), vec);
      if (vec)
        fmt::print("! odg write failed: {}\n", vec.message());
      else
        fmt::print("#ocp deglitch {} us\n", x);
    } else if (sscanf(line, "g %f %f", &a, &b) == 2) {
      g_kp.store(a);
      g_ki.store(b);
      g_reset_pi.store(true);
      fmt::print("#gains kp={:.5g} ki={:.5g}\n", a, b);
    } else if (strncmp(line, "mp", 2) == 0) {
      // mp <R> <L_uH> <lambda_pm> — observer motor params.
      float r = g_obs_r.load(), luh = g_obs_l.load() * 1e6f, lam = g_obs_lambda.load();
      sscanf(line, "mp %f %f %f", &r, &luh, &lam);
      g_obs_r.store(r);
      g_obs_l.store(luh * 1e-6f);
      g_obs_lambda.store(lam);
      g_reset_obs.store(true);
      fmt::print("#obs R={:.4g} ohm L={:.4g} uH lambda_pm={:.5g} Wb\n", r, luh, lam);
    } else if (strncmp(line, "og", 2) == 0) {
      // og <flux_gain> <pll_kp> <pll_ki> — observer/PLL gains.
      float fg = g_obs_flux_gain.load(), kp = g_obs_pll_kp.load(), ki = g_obs_pll_ki.load();
      sscanf(line, "og %f %f %f", &fg, &kp, &ki);
      g_obs_flux_gain.store(fg);
      g_obs_pll_kp.store(kp);
      g_obs_pll_ki.store(ki);
      g_reset_obs.store(true);
      fmt::print("#obs flux_gain={:.4g} pll_kp={:.4g} pll_ki={:.4g}\n", fg, kp, ki);
    } else if (sscanf(line, "e %d", &x) == 1) {
      uncoast();
      g_sampler.disable();
      g_mode.store(HOLD);
      g_id_target.store(0);
      g_iq_target.store(0);
      g_reset_pi.store(true);
      g_reset_obs.store(true);
      set_duties(kCenterDuty, kCenterDuty, kCenterDuty);
      g_sampler.set_edge(x != 0, logger);
      g_sampler.enable(); // recovery path after a trip/overrun
      fmt::print("#edge {} — armed, targets zeroed\n",
                 g_sampler.at_tep() ? "TEP (on_full)" : "TEZ (on_empty)");
    } else if (sscanf(line, "p %d %d", &x, &y) == 2 && x >= 0 && x < 3 && y >= 0 && y < 3 &&
               x != y) {
      g_auto_phase.store(false);
      g_sampler.set_phases(std::min(x, y), std::max(x, y));
      fmt::print("#phases x={} y={} (auto OFF)\n", g_sampler.phase_x(), g_sampler.phase_y());
    } else if (strncmp(line, "p auto", 6) == 0) {
      g_auto_phase.store(true);
      fmt::print("#phases auto ON\n");
    } else if (sscanf(line, "dbl %d", &x) == 1) {
      g_sampler.set_double_sample(x != 0);
      fmt::print("#dbl {}\n", x ? "ON — verification pair on the settled channel; expect "
                                  "dis~0 in 's' if the hop-settling fix holds"
                                : "OFF — single settled conversion (normal op)");
    } else if (sscanf(line, "m %d", &x) == 1 && (x == 1 || x == 2)) {
      g_sampler.set_samples_per_isr(x);
      fmt::print("#samples_per_isr {} ({})\n", x,
                 x == 1 ? "alternating phases, 50 us skew" : "both phases in one window");
    } else if (sscanf(line, "n %d", &x) == 1 && x >= 1 && x <= 64) {
      g_sampler.set_decimation((uint32_t)x);
      fmt::print("#decimation {} -> {:.1f} kHz sample+control rate\n", x, 20.0f / (float)x);
    } else if (line[0] == 'c') {
      g_sampler.dump_capture(); // last 256 samples: raw amps + isr_us per sample
    } else if (line[0] == 'z') {
      g_mode.store(HOLD); // zeroing brakes the bridge; disarm so nothing fights it
      g_id_target.store(0);
      g_iq_target.store(0);
      g_reset_pi.store(true);
      if (!g_sampler.zero_calibrate(logger)) {
        // The previous offsets were kept and everything was left disabled
        // (outputs off, sampler down) — nothing resumes on a failed zero.
        fmt::print("! zero calibration FAILED — previous offsets kept, outputs off, sampler "
                   "disabled ('e 1' re-arms after the sense path is fixed)\n");
      }
    } else if (line[0] == 'a') {
      const bool was_enabled = g_sampler.is_enabled();
      g_sampler.disable();
      std::this_thread::sleep_for(2ms);
      float amps[3];
      for (int p = 0; p < 3; p++)
        amps[p] =
            (g_sampler.read_raw_async(p) - g_sampler.raw_zero(p)) * g_sampler.amps_per_count();
      g_sampler.prime();
      if (was_enabled)
        g_sampler.enable();
      fmt::print("#async ia={:.3f} ib={:.3f} ic={:.3f} A\n", amps[0], amps[1], amps[2]);
    } else if (line[0] == 's') {
      auto st = g_sampler.stats();
      uint32_t runs = g_foc.task_runs.load();
      uint32_t notes = g_foc.notifications.load();
      if (st.count == 0) {
        fmt::print("#stats count=0 enabled={:d} — no samples taken\n",
                   g_sampler.is_enabled() ? 1 : 0);
      } else {
        // Filter counters are per phase A/B/C: imp = implausible (>12 A) held,
        // rej = in-band slew rejections, esc = escapes (filter followed a real
        // move), max|A| = largest raw conversion seen. Cleared here and on arm.
        fmt::print("#stats n={} en={:d} late={} ({:.1f}%) isr={:.1f}/{:.1f}/{:.1f}us | "
                   "imp={}/{}/{} rej={}/{}/{} esc={}/{}/{} max|A|={:.0f}/{:.0f}/{:.0f} | "
                   "foc runs={} coalesce={:.2f} cmax={:.0f}us\n",
                   st.count, g_sampler.is_enabled() ? 1 : 0, st.late,
                   100.0f * (float)st.late / (float)st.count, st.min_us, st.avg_us, st.max_us,
                   st.imp[0], st.imp[1], st.imp[2], st.rej[0], st.rej[1], st.rej[2], st.esc[0],
                   st.esc[1], st.esc[2], st.max_amps[0], st.max_amps[1], st.max_amps[2], runs,
                   runs ? (float)notes / (float)runs : 0.0f,
                   (float)g_foc.compute_cyc_max.load() / 240.0f);
        if (g_sampler.double_sample()) {
          auto db = g_sampler.dbl_stats();
          fmt::print("#dbl pairs={} dis={} spike: agree={} first={} second={} maxD={:.1f}A\n",
                     db.pairs, db.dis, db.spike_agree, db.spike_first, db.spike_second,
                     db.max_delta_amps);
        }
      }
      g_sampler.reset_stats();
      g_foc.compute_cyc_max.store(0);
    } else if (line[0] == 'r') {
      std::error_code rec;
      auto regs = bsp.gate_driver()->read_all_registers(rec);
      if (rec) {
        fmt::print("#err register read failed: {}\n", rec.message());
      } else {
        fmt::print("#regs fault1=0x{:04x} vgs2=0x{:04x} drv_ctl=0x{:04x} gate_hs=0x{:04x} "
                   "gate_ls=0x{:04x} ocp=0x{:04x} csa=0x{:04x} nfault_active={:d}\n",
                   regs.fault_status_1, regs.vgs_status_2, regs.driver_control, regs.gate_drive_hs,
                   regs.gate_drive_ls, regs.ocp_control, regs.csa_control,
                   bsp.gate_driver()->fault_pin_active() ? 1 : 0);
      }
    } else if (line[0] == 'f') {
      // Disarm: back to HOLD, zero targets, reset PI. The loop keeps running
      // (sampler stays enabled) but commands 0 V -> centered duties -> no current.
      g_mode.store(HOLD);
      g_id_target.store(0);
      g_iq_target.store(0);
      g_reset_pi.store(true);
      fmt::print("#disarmed (id*=iq*=0)\n");
    } else {
      fmt::print("#err unknown command\n");
    }
  }
}
