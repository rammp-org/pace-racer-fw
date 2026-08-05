// Sensorless FOC — stage 3: flux observer + PLL, logged against the halls.
//
// Stages 0-2 built the synchronized sampler, the CPU1 current loop, and open-loop
// I/f spin. Stage 3 adds a flux-linkage observer (foc_observer.hpp) that runs in
// the FOC task every tick with NO control authority: the I/f synthetic angle
// still drives the SVPWM. The observer estimates the rotor angle/speed from the
// electrical model (measured i, commanded v, and R/L/lambda_pm), and the stream
// logs theta_est next to theta_hall (the on-board halls, ground truth) so we can
// see how well it tracks and DOWN TO WHAT SPEED before it falls apart — that
// speed is the I/f->observer handoff point for stage 4.
//
// Test: `run 4 100 3` (spin), watch derr (theta_est - theta_hall). If derr holds
// roughly constant (a fixed offset is just hall mounting) and rpm_est ~ rpm_hall,
// the observer tracks. Tune lambda_pm with 'mp', the PLL/flux gains with 'og',
// from the logs — zero risk since nothing steers on the estimate yet.
//
// The observer uses COMMANDED voltage, whose deadtime error dominates at low
// speed where back-EMF is small — expect it to degrade below some rpm. That is
// the measurement, not a bug.
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

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "logger.hpp"
#include "pace-racer-board.hpp"
#include "task.hpp"

#include "fast_math.hpp" // espp::fast_sin / fast_cos

#include "foc_math.hpp"
#include "foc_observer.hpp"
#include "foc_sampler.hpp"
#include "hall_sensor.hpp"

using namespace std::chrono_literals;

static constexpr float kTwoPi = 6.28318530718f;

// Bring-up runs on 24 V, not 48 V: switching ripple scales with bus voltage and
// L is only 94 uH/phase, so halving the bus halves the ripple on a 10 A command.
// MUST match the bench supply: SVPWM scales duties by it, so the observer's
// commanded-voltage model is only right when this is the real bus voltage.
// (24 V was the bring-up value; 48 V doubles ripple to ~6-7 App on 94 uH —
// watch the LM75s. Re-zero with 'z' after changing supplies.)
static constexpr float kBusVoltage = 48.0f;
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
static constexpr float kHardMaxTargetAmps = 25.0f;
static constexpr float kHardMaxTripAmps = 30.0f; // plausibility rides at +3 -> 33 A
static constexpr float kHardMaxVolts = 24.0f;    // 'vl' ceiling: half the 48 V bus
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
static constexpr float kHoDecayAps = 1.0f;    // iq decay rate during CONVERGE
// Absolute CONVERGE floor. A FRACTIONAL floor (25% of run amps) made
// convergence impossible at high run amps on the dyno: the load angle only
// reaches 90° when iq decays to ~the machine's actual drag current (sinγ =
// T/(k·λ·I)), and drag doesn't scale with the commanded amps — at `run 10` the
// 2.5 A floor parked aerr at ~90° until the stuck-abort fired, forever. 0.5 A
// matches the unloaded converge current observed on this rig.
static constexpr float kIqFloorAmps = 0.5f;
static constexpr float kConvergeDeg = 15.0f;  // frames agree inside this cone
static constexpr uint32_t kEngageTicks = 1000;    // 50 ms sustained above engage rpm
static constexpr uint32_t kConvergeTicks = 400;   // 20 ms inside the cone -> switch
static constexpr uint32_t kConvStuckTicks = 10000; // 500 ms at floor unconverged -> abort
static constexpr uint32_t kCooldownTicks = 10000; // 500 ms after revert/abort: no chatter
// Outer speed loop, active only in CLOSED: rpm setpoint (ramped at the run's
// ramp rate) -> iq command, clamped to ±run amps (amps = torque ceiling).
// Few-Hz loop around a 400 rad/s current loop — decades of separation.
static constexpr float kSpeedKp = 0.05f; // A per rpm
static constexpr float kSpeedKi = 0.2f;  // A per rpm-second
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
static constexpr float kCenterDuty = 0.5f; // SVPWM idle: all phases at 0.5 = no current
static constexpr float kPwmPeriodUs = 50.0f; // espp BldcDriver is fixed at 20 kHz

// dq current-loop voltage limit. The loop output is a voltage; this caps it so a
// bad command or gain can't slam the full bus. 8 V into ~0.15 ohm is ~50 A DC,
// but the loop is regulating current, so this is just the ceiling — start here.
static constexpr float kVoltageLimit = 8.0f;

// Current-loop PI gains. Halved from current-control's tuning: that fit the
// series A->B pair (2R, 2L); the dq plant is a single phase (R, L). 169 rad/s
// bandwidth (auto-tuner's verified-overdamped crossover for the NineBot S):
// kp = 169*336uH = 0.0568, ki = kp*R/L = 0.0568*961 = 54.5.
static constexpr float kDefaultKp = 0.0568f; // V/A
static constexpr float kDefaultKi = 54.5f;   // V/(A*s)

// I/f open-loop startup (stage 2b). 15 pole pairs (measured: 90 hall edges/rev).
static constexpr int kPolePairs = 15;
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
static constexpr float kObsR = 0.323f;
static constexpr float kObsL = 336.0e-6f;
// Measured on the NineBot S 2026-07-27 (I/f spin, 'flux' column): 0.025-0.027.
static constexpr float kObsLambda = 0.026f;
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
struct FocState {
  std::atomic<uint32_t> seq{0};
  FocSnapshot snap{};

  std::atomic<uint32_t> task_runs{0};     // FOC-task wakeups
  std::atomic<uint32_t> notifications{0}; // sum of ulTaskNotifyTake() returns
  std::atomic<uint32_t> compute_cyc_max{0};

  void publish(const FocSnapshot &s) {
    const uint32_t v = seq.load(std::memory_order_relaxed);
    seq.store(v + 1, std::memory_order_relaxed); // odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    snap = s;
    std::atomic_thread_fence(std::memory_order_release);
    seq.store(v + 2, std::memory_order_release); // even: stable
  }
  FocSnapshot read() const {
    FocSnapshot s;
    uint32_t before, after;
    do {
      before = seq.load(std::memory_order_acquire);
      std::atomic_thread_fence(std::memory_order_acquire);
      s = snap;
      std::atomic_thread_fence(std::memory_order_acquire);
      after = seq.load(std::memory_order_acquire);
    } while ((before & 1u) || before != after);
    return s;
  }
};

sensorless::FocSampler g_sampler;
sensorless::FluxObserver g_observer;
FocState g_foc;
TaskHandle_t g_foc_task = nullptr;
std::shared_ptr<espp::BldcDriver> g_driver;
HallSensor *g_hall = nullptr; // ground-truth angle, read in the stream task only

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
enum Mode { HOLD = 0, SPIN = 1, STOPPING = 2 };
std::atomic<int> g_mode{HOLD};
// Handoff parameters ('ho <engage_rpm> <revert_rpm> <decay_A_per_s>').
std::atomic<float> g_ho_engage{kHoEngageRpm};
std::atomic<float> g_ho_revert{kHoRevertRpm};
std::atomic<float> g_ho_decay{kHoDecayAps};
// Speed-loop gains ('sg <kp> <ki>').
std::atomic<float> g_spd_kp{kSpeedKp};
std::atomic<float> g_spd_ki{kSpeedKi};
// Escalation limits ('lim', 'vl', 'tl') — see the hard-ceiling constants.
std::atomic<float> g_max_target{kMaxTargetAmps};
std::atomic<float> g_vlim{kVoltageLimit};
std::atomic<float> g_temp_limit{kTempLimitC};
// FOC task -> stream/console handshakes (SPI can't run on the control task).
std::atomic<bool> g_request_coast{false}; // apply DRV8353 Hi-Z coast
std::atomic<bool> g_failsafe_msg{false};  // observer fail-safe fired, report it
std::atomic<bool> g_ceiling_msg{false};   // speed collapsed with iq pinned at the clamp
std::atomic<bool> g_coasting{false};      // outputs are Hi-Z; drive cmds must clear
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
void foc_task_fn(void *) {
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
    if (dt <= 0.0f || dt > 0.01f) dt = 50.0e-6f; // guard the first tick / long gaps
    last_cyc = c0;
    have_last = true;

    pi_d.kp = pi_q.kp = g_kp.load(std::memory_order_relaxed);
    pi_d.ki = pi_q.ki = g_ki.load(std::memory_order_relaxed);
    pi_d.lim = pi_q.lim = g_vlim.load(std::memory_order_relaxed);
    if (g_reset_pi.exchange(false)) {
      pi_d.reset();
      pi_q.reset();
    }
    if (g_reset_obs.exchange(false)) g_observer.reset();

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
    const int mode = g_mode.load(std::memory_order_relaxed);
    if (mode == SPIN && prev_mode != SPIN) { // entering spin: reset the ramp
      sp_theta = 0.0f;
      sp_omega = 0.0f;
      sp_align_left = g_spin_align_ticks.load(std::memory_order_relaxed);
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
        if (cooldown) cooldown--;
        static float spd_set = 0.0f, spd_integ = 0.0f; // speed loop state
        if (ho == CLOSED) { // observer drives; synthetic state shadows it for revert
          th = th_est;
          sp_theta = th_est;
          sp_omega = g_observer.omega();
          // Outer speed loop: ramp the rpm setpoint toward the run target at
          // the run's ramp rate, PI to an iq command bounded by ±amps. Seeded
          // at the CONVERGE exit (setpoint = actual rpm, integrator = decayed
          // iq), so the transfer is bumpless.
          const float rpm_t =
              g_spin_omega_target.load(std::memory_order_relaxed) / kRpmToOmegaE;
          const float step =
              g_spin_ramp_rate.load(std::memory_order_relaxed) / kRpmToOmegaE * dt;
          spd_set += std::clamp(rpm_t - spd_set, -step, step);
          const float serr = spd_set - rpm_est;
          spd_integ = std::clamp(
              spd_integ + g_spd_ki.load(std::memory_order_relaxed) * serr * dt, -amps, amps);
          iq_cmd = std::clamp(g_spd_kp.load(std::memory_order_relaxed) * serr + spd_integ,
                              -amps, amps);
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
              cooldown = kCooldownTicks;
            }
          }
        } else { // synthetic I/f ramp keeps running under IF_DRIVE and CONVERGE
          const float wt = g_spin_omega_target.load(std::memory_order_relaxed);
          sp_omega =
              std::min(sp_omega + g_spin_ramp_rate.load(std::memory_order_relaxed) * dt, wt);
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
            if (engage_ok >= kEngageTicks) {
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
            if (iq_cmd <= floor_a) conv_stuck++;
            if (conv_ok >= kConvergeTicks) { // frames agree: switching is a near-no-op
              ho = CLOSED;
              th = th_est;
              sp_theta = th_est;
              sp_omega = g_observer.omega();
              spd_set = rpm_est;    // bumpless speed-loop entry: zero error,
              spd_integ = iq_cmd;   // integrator holds the converged torque
            } else if (rpm_est < revert || conv_stuck >= kConvStuckTicks) {
              ho = IF_DRIVE; // didn't converge / estimate collapsed: back to stiff I/f
              cooldown = kCooldownTicks;
            }
          }
        }
        // Fail-safe: the observer has (or is gaining) authority and its flux
        // estimate left the physical band — the rotor is lost. Coast, never
        // fight a bad angle with real current.
        if (ho != IF_DRIVE) {
          flux_bad = flux_ok ? 0 : flux_bad + 1;
          if (flux_bad >= kFluxFailTicks) {
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
    } else if (mode == STOPPING) {
      sp_theta += sp_omega * dt; // keep the frame rotating: no dq frame jump
      sp_theta -= kTwoPi * floorf(sp_theta / kTwoPi);
      th = sp_theta;
      idref = 0.0f;
      iqref = 0.0f;
      mstate = 'X';
      if (++stop_ticks == kStopUnloadTicks) g_request_coast.store(true);
    } else { // HOLD
      th = g_theta.load(std::memory_order_relaxed);
      th -= kTwoPi * floorf(th / kTwoPi);
      idref = g_id_target.load(std::memory_order_relaxed);
      iqref = g_iq_target.load(std::memory_order_relaxed);
      sp_omega = 0.0f;
    }
    last_th = th;

    const float s = espp::fast_sin(th);
    const float c = espp::fast_cos(th);
    const auto dq = foc::park(ab, s, c);

    // Two PI current loops -> dq voltage -> inverse Park -> SVPWM -> duties.
    const float vd = pi_d.step(idref - dq.d, dt);
    const float vq = pi_q.step(iqref - dq.q, dt);
    const auto vab = foc::inverse_park({vd, vq}, s, c);
    const auto duties = foc::svpwm(vab, kBusVoltage, kMinDuty, kMaxDuty);
    g_driver->set_pwm(duties.a, duties.b, duties.c);
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
                   g_observer.theta() * kRadToDeg, g_observer.omega() / kRpmToOmegaE,
                   g_observer.flux_mag(), mstate});

    const uint32_t dt_cyc = esp_cpu_get_cycle_count() - c0;
    if (dt_cyc > g_foc.compute_cyc_max.load(std::memory_order_relaxed))
      g_foc.compute_cyc_max.store(dt_cyc, std::memory_order_relaxed);
    g_foc.task_runs.fetch_add(1, std::memory_order_relaxed);
    g_foc.notifications.fetch_add(got, std::memory_order_relaxed);
  }
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

  {
    std::error_code ec;
    // ESP32 reboot does NOT power-cycle the DRV8353 — clear any latched faults.
    bsp.gate_driver()->clear_faults(ec);
    if (ec) logger.error("Failed to clear DRV8353 faults: {}", ec.message());
  }

  // Hardware protection + known-good driver state. Force-write DRIVER_CONTROL
  // (not read-modify-write, so a poisoned COAST bit can't survive), CSA GAIN_20.
  {
    using GD = Bsp::GateDriver;
    std::error_code ec;
    auto gd = bsp.gate_driver();
    GD::DriverControl drv_ctl{.raw = 0x0000};
    bool ok = gd->write_driver_control(drv_ctl, ec) &&
              gd->set_ocp_mode(GD::OcpMode::LATCHED_SHUTDOWN, ec) &&
              gd->set_ocp_deglitch(GD::OcpDeglitch::US_4, ec) &&
              gd->set_vds_level(GD::VdsLevel::V_0_06, ec) && // ~17-22 A hardware backstop
              gd->set_sense_overcurrent_enabled(true, ec) &&
              gd->set_sense_level(GD::SenseLevel::V_0_25, ec) &&
              gd->set_csa_gain(GD::CsaGain::GAIN_20, ec);
    gd->clear_faults(ec);
    auto ocp = gd->read_ocp_control(ec);
    auto csa = gd->read_csa_control(ec);
    auto dcv = gd->read_driver_control(ec);
    if (!ok || ec || dcv.coast() || dcv.brake() ||
        ocp.ocp_mode() != GD::OcpMode::LATCHED_SHUTDOWN ||
        ocp.deglitch() != GD::OcpDeglitch::US_4 || ocp.vds_level() != GD::VdsLevel::V_0_06 ||
        csa.sense_overcurrent_disabled() || csa.sense_level() != GD::SenseLevel::V_0_25 ||
        csa.gain() != GD::CsaGain::GAIN_20) {
      logger.error("Failed to configure DRV8353 (ocp=0x{:04x} csa=0x{:04x} drv=0x{:04x}) — not "
                   "running. Is VM on?",
                   ocp.raw, csa.raw, dcv.raw);
      return;
    }
    logger.info("DRV8353 armed: CSA GAIN_20, VDS 0.06 V (~17-22 A), SEN 0.25 V, latched");
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
  xTaskCreatePinnedToCore(foc_task_fn, "foc", 4096, nullptr, 20, &g_foc_task, 1);
  g_sampler.set_notify_task(g_foc_task);

  driver->enable();
  driver->set_pwm(kCenterDuty, kCenterDuty, kCenterDuty);

  if (!g_sampler.zero_calibrate(logger)) {
    logger.error("Zero-current reading is noisy or railed — sensing broken, not running");
    return;
  }
  driver->set_pwm(kCenterDuty, kCenterDuty, kCenterDuty);

  if (!g_sampler.set_edge(true /*TEP*/, logger)) return;
  driver->set_pwm(kCenterDuty, kCenterDuty, kCenterDuty);
  // Enabling the sampler starts the FOC task's control loop. With Id*=Iq*=0 the
  // loop commands 0 V -> centered duties -> no current, so this is safe.
  g_sampler.enable();

  // Measure phases A and B, reconstruct C (its sense is noisy under rotation).
  g_sampler.set_phases(0, 1);

  // Hall sensor: ground-truth angle for validating the observer. Never used in
  // the control path — it exists here only to log theta_hall alongside theta_est.
  static HallSensor hall({.pin_a = kHallA, .pin_b = kHallB, .pin_c = kHallC,
                          .pole_pairs = kPolePairs});
  hall.init();
  g_hall = &hall;

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
  fmt::print("%t, mode, id, iq, iqref, vd, vq, aerr, rpm_drive, rpm_est, rpm_hall, flux, "
             "t0, t1, t2, t3\n");
  auto stream_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::steady_clock::now();
    static int tick = 0;
    static std::array<float, 4> temps = {NAN, NAN, NAN, NAN};
    ++tick;

    // Board temperatures every 5th tick (2 Hz — LM75 conversion is ~100 ms and
    // the I2C traffic, like the DRV fault poll, briefly delays the sampling ISR;
    // late-rejection covers it). The thermal guard fires the same gentle
    // STOPPING path as 'stop' — never a brake — so an unattended soak ends as a
    // clean unload + coast with the stream still telling the story.
    if (tick % 5 == 0 && bsp.temperature_sensors_initialized()) {
      Bsp::TemperatureErrors terrs;
      auto tr = bsp.board_temperatures_c(terrs);
      int hot = -1;
      const float tl = g_temp_limit.load(std::memory_order_relaxed);
      for (size_t i = 0; i < temps.size(); i++) {
        temps[i] = terrs[i] ? NAN : tr[i];
        if (!terrs[i] && tr[i] > tl) hot = (int)i;
      }
      const int md = g_mode.load(std::memory_order_relaxed);
      const bool driving = g_sampler.is_enabled() &&
                           (md == SPIN || g_id_target.load() != 0.0f || g_iq_target.load() != 0.0f);
      if (hot >= 0 && driving) {
        g_id_target.store(0);
        g_iq_target.store(0);
        g_mode.store(STOPPING);
        fmt::print("! thermal guard: T{}={:.1f}C > {:.0f}C limit — unloading to 0 A, then Hi-Z "
                   "coast ('tl' to raise)\n",
                   hot, temps[hot], tl);
      }
    }

    if (g_sampler.take_tripped()) {
      auto ti = g_sampler.trip_info();
      auto st = g_sampler.stats();
      // Report only — the FOC task already switched itself to STOPPING
      // (closed-loop unload -> Hi-Z coast). No brake, no disarm, no frame jump.
      // Everything needed to call glitch-vs-real: which limit fired, filtered
      // vs raw at the trip tick, and the filter counters since arm.
      fmt::print("! soft trip >{:.0f}A on {}{} — unloading to 0 A, then Hi-Z coast "
                 "(any drive command re-engages)\n"
                 "!  filtered A={:+.1f} B={:+.1f} C={:+.1f}  raw {}={:+.1f}\n"
                 "!  since arm: imp={}/{}/{} rej={}/{}/{} esc={}/{}/{} max|A|={:.0f}/{:.0f}/{:.0f}\n",
                 g_sampler.trip_amps(), "ABC"[ti.trip_phase],
                 ti.trip_phase == ti.recon ? " (recon)" : "",
                 ti.amps[0], ti.amps[1], ti.amps[2], "ABC"[ti.raw_phase], ti.raw_amps, st.imp[0],
                 st.imp[1], st.imp[2], st.rej[0], st.rej[1], st.rej[2], st.esc[0], st.esc[1],
                 st.esc[2], st.max_amps[0], st.max_amps[1], st.max_amps[2]);
      g_sampler.dump_summary();
    }
    if (g_sampler.take_overran()) {
      fmt::print("! sampler ISR exceeded budget — disabled. 'e 1' to re-enable\n");
    }
    // Coast handshake: the FOC task can't touch SPI, so it requests Hi-Z here
    // (end of a gentle stop, or the observer fail-safe).
    if (g_request_coast.exchange(false)) {
      std::error_code cec;
      bsp.gate_driver()->set_coast(true, cec);
      if (cec) {
        fmt::print("! coast SPI write failed: {} — outputs still live\n", cec.message());
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
    // DRV fault poll every ~5 s: the SPI transaction briefly stalls the sampling
    // ISR on CPU0 (the sampler's late-rejection handles the resulting late
    // samples, but keeping it infrequent means fewer to reject).
    if (tick % 50 == 0) {
      std::error_code fec;
      auto fs = bsp.gate_driver()->fault_status(fec);
      if (!fec && fs.raw != 0) {
        g_sampler.disable();
        g_mode.store(HOLD);
        g_id_target.store(0);
        g_iq_target.store(0);
        driver->set_pwm(kCenterDuty, kCenterDuty, kCenterDuty);
        fmt::print("! DRV8353 fault 0x{:04x} — disarmed (power-cycle VM to clear)\n", fs.raw);
      }
    }

    // Silent while disarmed — an idle bench prints nothing; trips, faults, and
    // command echoes are the only unsolicited output.
    if (g_sampler.is_enabled()) {
      float seconds =
          std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();

      // Hall ground truth (electrical angle in 60 deg sectors, mechanical rpm).
      std::error_code hec;
      g_hall->update(hec);
      float th_hall = g_hall->get_radians() * 180.0f / 3.14159265f;
      float rpm_hall = g_hall->get_rpm();

      auto sn = g_foc.read();
      // Observer-vs-drive angle, folded to [-180, 180) — the CONVERGE signal.
      float aerr = sn.th_est_deg - sn.th_drive_deg;
      aerr -= 360.0f * floorf(aerr / 360.0f + 0.5f);
      (void)th_hall; // rpm_hall is the hall ground-truth column now
      fmt::print("{:.2f}, {}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.0f}, {:.0f}, {:.0f}, "
                 "{:.0f}, {:.4f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}\n",
                 seconds, sn.state, sn.id, sn.iq, sn.iqref, sn.vd, sn.vq, aerr, sn.rpm_drive,
                 sn.rpm_est, rpm_hall, sn.flux_mag, temps[0], temps[1], temps[2], temps[3]);
    }
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 100ms);
    return false;
  };
  auto stream_task = espp::Task({.callback = stream_fn,
                                 .task_config = {.name = "stream", .stack_size_bytes = 8 * 1024},
                                 .log_level = espp::Logger::Verbosity::WARN});
  stream_task.start();

  setvbuf(stdin, nullptr, _IONBF, 0);
  fmt::print("#ready bus={:.1f}V pwm=20kHz. Sensorless FOC + high-power instrumentation.\n"
             "#  SPIN: run [<A> <rpm> <ramp_s>] | stop   HOLD: th/id/iq <val>\n"
             "#  g <kp> <ki> | mp <R> <L_uH> <lam> | og <fluxgain> <pll_kp> <pll_ki> | f | e 1\n"
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
    int ch = getchar();
    if (ch == EOF) {
      std::this_thread::sleep_for(50ms);
      continue;
    }
    if (ch != '\n' && ch != '\r') {
      if (len < sizeof(line) - 1) line[len++] = (char)ch;
      continue;
    }
    if (len == 0) continue;
    line[len] = '\0';
    len = 0;

    // Any drive command must first release a hardware Hi-Z coast.
    auto uncoast = [&]() {
      if (g_coasting.exchange(false)) {
        std::error_code cec;
        bsp.gate_driver()->set_coast(false, cec);
        if (cec) fmt::print("! uncoast SPI write failed: {}\n", cec.message());
        g_reset_pi.store(true);
      }
    };

    float a = 0, b = 0;
    int x = 0, y = 0;
    if (strncmp(line, "run", 3) == 0) {
      // run [<amps> <rpm> <ramp_s>] — I/f open-loop spin. Defaults are gentle.
      float amps = kSpinAmps, rpm = kSpinRpm, ramp = kSpinRampS;
      sscanf(line, "run %f %f %f", &amps, &rpm, &ramp);
      amps = std::clamp(amps, 0.0f, g_max_target.load());
      if (ramp < 0.1f) ramp = 0.1f;
      const float wt = rpm * kRpmToOmegaE;
      g_spin_amps.store(amps);
      g_spin_omega_target.store(wt);
      g_spin_ramp_rate.store(wt / ramp);
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
    } else if (sscanf(line, "id %f", &a) == 1) {
      if (!g_sampler.is_enabled())
        fmt::print("! sampler disabled — 'e 1' to enable before commanding current\n");
      uncoast();
      g_mode.store(HOLD);
      g_id_target.store(std::clamp(a, -g_max_target.load(), g_max_target.load()));
      fmt::print("#id* {:.3f} A\n", g_id_target.load());
    } else if (sscanf(line, "iq %f", &a) == 1) {
      if (!g_sampler.is_enabled())
        fmt::print("! sampler disabled — 'e 1' to enable before commanding current\n");
      uncoast();
      g_mode.store(HOLD);
      g_iq_target.store(std::clamp(a, -g_max_target.load(), g_max_target.load()));
      fmt::print("#iq* {:.3f} A\n", g_iq_target.load());
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
      fmt::print("#lim target={:.1f}A trip={:.1f}A plausible={:.1f}A (hard ceilings {:.0f}/{:.0f})\n",
                 a, b, b + 3.0f, kHardMaxTargetAmps, kHardMaxTripAmps);
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
    } else if (sscanf(line, "vds %d", &x) == 1 && x >= 0 && x <= 4) {
      // vds <0-4> — DRV8353 hardware OCP threshold, fine steps only:
      // 0.06/0.07/0.08/0.09/0.10 V across Rdson. The next enum step (0.20 V)
      // is past the ADC rail and the FETs' sane region — not offered. Rdson
      // rises ~1.5-2x with junction temp, so the amp threshold DERATES as the
      // FETs heat: late-run OCP trips at lower current are expected physics.
      static constexpr const char *kVdsEstAmps[5] = {"17-22", "20-26", "23-30", "26-33", "29-37"};
      std::error_code vec;
      auto gd = bsp.gate_driver();
      gd->set_vds_level(static_cast<Bsp::GateDriver::VdsLevel>(x), vec);
      auto ocp = gd->read_ocp_control(vec);
      if (vec || (int)ocp.vds_level() != x) {
        fmt::print("! vds write failed (ocp=0x{:04x}{}{})\n", ocp.raw, vec ? ", " : "",
                   vec ? vec.message() : "");
      } else {
        fmt::print("#vds {:.2f}V (~{} A cold, derates hot)\n", 0.06f + 0.01f * (float)x,
                   kVdsEstAmps[x]);
      }
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
      driver->set_pwm(kCenterDuty, kCenterDuty, kCenterDuty);
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
      g_sampler.zero_calibrate(logger);
    } else if (line[0] == 'a') {
      const bool was_enabled = g_sampler.is_enabled();
      g_sampler.disable();
      std::this_thread::sleep_for(2ms);
      float amps[3];
      for (int p = 0; p < 3; p++)
        amps[p] = (g_sampler.read_raw_async(p) - g_sampler.raw_zero(p)) * g_sampler.amps_per_count();
      g_sampler.prime();
      if (was_enabled) g_sampler.enable();
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
