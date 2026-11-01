// Host test for the D40 changes: the profile acceleration ceiling as the
// supervisor applies it (ARMED velocity, SAFE_STOPPING, HOLDING), and the
// rising-only iq slew in foc_math.hpp. Built once per motor profile.

#include <cmath>
#include <cstdio>
#include <string_view>

#include "drive_supervisor.hpp"
#include "foc_math.hpp"

static int g_fail = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
      g_fail++;                                                                                    \
    }                                                                                              \
  } while (0)
static bool near(float a, float b, float tol = 1e-3f) { return std::fabs(a - b) <= tol; }

// The plant as the supervisor sees it, and what its hooks last asked for.
struct Fake {
  supervisor::Measured m;
  float vel_rpm{NAN}, vel_ramp{NAN};
  float pos_maxrpm{NAN};
  int velocity_calls{0}, position_calls{0};
};

static supervisor::Supervisor make(Fake &f) {
  supervisor::Hooks h;
  h.coast = [] {};
  h.uncoast = [] {};
  h.idle = [] {};
  h.reset_controllers = [] {};
  h.torque = [](float, float, float) {};
  h.velocity = [&f](float rpm, float ramp, float, float) {
    f.vel_rpm = rpm;
    f.vel_ramp = ramp;
    f.velocity_calls++;
  };
  h.position = [&f](float, float maxrpm, float, bool) {
    f.pos_maxrpm = maxrpm;
    f.position_calls++;
  };
  h.measure = [&f] { return f.m; };
  return supervisor::Supervisor(std::move(h), rammp::AxisId::DRIVE_LEFT);
}

static rammp::MotorCommand vel_cmd(uint8_t seq, float rad_s, float accel) {
  rammp::MotorCommand c{};
  c.seq = seq;
  c.requested_state = rammp::RequestedState::ARMED;
  c.mode = rammp::ControlMode::VELOCITY;
  c.velocity = rad_s;
  c.accel_limit = accel;
  return c;
}

static void tick(supervisor::Supervisor &s) {
  g_fake_now_us += 10000; // the 100 Hz supervisor tick
  s.tick();
}

static void test_accel_ceiling() {
  Fake f;
  auto s = make(f);
  const float ceil = motor::kMaxAccelRpmS; // rpm/s, 0 = none
  g_fake_now_us = 1000000;

  // accel_limit 0: the ceiling where the profile has one, else a step.
  s.on_command(vel_cmd(1, 3.0f, 0.0f));
  tick(s);
  CHECK(s.board_state() == rammp::BoardState::ARMED);
  CHECK(near(f.vel_rpm, 3.0f * motor::kRadSToRpm));
  if (ceil > 0) {
    CHECK(near(f.vel_ramp, ceil, 0.01f));
    CHECK(near(s.state().accel_limit_eff, ceil * motor::kRpmToRadS));
  } else {
    CHECK(f.vel_ramp >= 1.0e5f);
    CHECK(s.state().accel_limit_eff == 0.0f);
  }

  // A request under the ceiling passes; one over it is clamped to it.
  s.on_command(vel_cmd(2, 3.0f, 5.0f));
  tick(s);
  CHECK(near(f.vel_ramp, 5.0f * motor::kRadSToRpm, 0.01f));
  s.on_command(vel_cmd(3, 3.0f, 100.0f));
  tick(s);
  CHECK(near(f.vel_ramp, ceil > 0 ? ceil : 100.0f * motor::kRadSToRpm, 0.01f));

  // hub20: the ceiling sits above the MIB NORMAL worst case per wheel
  // (1.0 m/s^2 linear + 2.0 rad/s^2 yaw, 0.254 m wheel, 0.558 m track).
  if (ceil > 0 && motor::kName == std::string_view("hub20")) {
    const float r = 0.127f, half_track = 0.279f;
    const float normal_rad_s2 = (1.0f + 2.0f * half_track) / r;
    CHECK(ceil * motor::kRpmToRadS > normal_rad_s2);
  }

  // SAFE_STOP with accel_limit 0: the stop-path ramp, not the drive ceiling.
  auto stop = vel_cmd(4, 3.0f, 0.0f);
  stop.requested_state = rammp::RequestedState::SAFE_STOP;
  f.m.velocity_rad_s = 3.0f;
  s.on_command(stop);
  tick(s);
  CHECK(s.board_state() == rammp::BoardState::SAFE_STOPPING);
  CHECK(near(f.vel_rpm, 0.0f));
  CHECK(near(f.vel_ramp, motor::kSafeStopRampRpmS, 0.01f));

  // Stopped: HOLDING runs the position loop, which has no setpoint ramp.
  f.m.velocity_rad_s = 0.0f;
  stop.seq = 5;
  s.on_command(stop);
  tick(s);
  CHECK(s.board_state() == rammp::BoardState::HOLDING);
  CHECK(f.position_calls > 0);
  CHECK(near(f.pos_maxrpm, motor::kHoldMaxRpm));
}

static void test_slew_rising() {
  using foc::slew_rising;
  // Off: anything passes.
  CHECK(slew_rising(30.0f, 0.0f, 0.0f) == 30.0f);
  // Rising either way is limited to one step.
  CHECK(near(slew_rising(30.0f, 0.0f, 0.1f), 0.1f));
  CHECK(near(slew_rising(30.0f, 5.0f, 0.1f), 5.1f));
  CHECK(near(slew_rising(-30.0f, -5.0f, 0.1f), -5.1f));
  // Toward zero is immediate.
  CHECK(slew_rising(2.0f, 30.0f, 0.1f) == 2.0f);
  CHECK(slew_rising(0.0f, -30.0f, 0.1f) == 0.0f);
  // A sign change drops to zero at once, then rises.
  CHECK(near(slew_rising(-30.0f, 20.0f, 0.1f), -0.1f));
  CHECK(near(slew_rising(30.0f, -20.0f, 0.1f), 0.1f));
  // Inside the step: exact.
  CHECK(slew_rising(5.05f, 5.0f, 0.1f) == 5.05f);

  // The profile rate at the 20 kHz tick: a full kMaxAmps rise takes
  // kMaxAmps / kIqSlewAPerMs ms, and the unload back to 0 one tick.
  if (motor::kIqSlewAPerMs > 0) {
    const float dt = 50.0e-6f, step = motor::kIqSlewAPerMs * 1000.0f * dt;
    float iq = 0.0f;
    int ticks = 0;
    while (iq < motor::kMaxAmps && ticks < 100000) {
      iq = slew_rising(motor::kMaxAmps, iq, step);
      ticks++;
    }
    const float ms = ticks * dt * 1000.0f;
    CHECK(near(ms, motor::kMaxAmps / motor::kIqSlewAPerMs, 0.1f));
    std::printf("  %s: 0 -> %.0f A in %.2f ms at %.1f A/ms\n", motor::kName, motor::kMaxAmps, ms,
                motor::kIqSlewAPerMs);
    CHECK(slew_rising(0.0f, iq, step) == 0.0f);
  }
}

int main() {
  std::printf("profile %s: accel ceiling %.0f rpm/s, iq slew %.1f A/ms\n", motor::kName,
              motor::kMaxAccelRpmS, motor::kIqSlewAPerMs);
  test_accel_ceiling();
  test_slew_rising();
  std::printf("%s\n", g_fail ? "FAILED" : "ok");
  return g_fail ? 1 : 0;
}
