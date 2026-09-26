// Host-side stand-in for the MIB: one RTPS participant that publishes
// MotorCommand to a PACE RACER axis and prints the MotorState it sends back.
//
// The message structs come straight from rammp-rtps, so espp's reflection lays
// out the wire bytes identically on both ends. Nothing here decodes by hand.
//
//   mib_stub <interface-ip> <axis> [seconds]
//
// <axis> is the rammp segment name (drive_left, drive_right, front_caster_left,
// front_caster_right). Commands come from stdin, one per line, so a scripted
// session is just a here-doc:
//
//   arm | disarm | stop | estop      requested_state
//   coast | hold                     mode with no setpoint
//   torque <N.m> | vel <rad/s> | pos <rad>
//   lim <torque N.m> <vel rad/s> <accel rad/s^2>   0 = firmware ceiling / no ramp
//   clr                              bump clear_fault_req_id
//   wait <s>                         keep streaming, say nothing
//   q                                stop streaming and exit
//
// The command stream runs at 20 Hz whether or not anything was typed, because
// the board's 200 ms watchdog treats silence as a fault. Every received
// MotorState is summarized once a second; pass -v to print every one.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include "messages/motor_message.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"

namespace {

const char *state_name(rammp::BoardState s) {
  switch (s) {
  case rammp::BoardState::DISARMED: return "DISARMED";
  case rammp::BoardState::ARMED: return "ARMED";
  case rammp::BoardState::SAFE_STOPPING: return "SAFE_STOPPING";
  case rammp::BoardState::HOLDING: return "HOLDING";
  case rammp::BoardState::FAULT: return "FAULT";
  }
  return "?";
}
const char *mode_name(rammp::ControlMode m) {
  switch (m) {
  case rammp::ControlMode::COAST: return "COAST";
  case rammp::ControlMode::TORQUE: return "TORQUE";
  case rammp::ControlMode::VELOCITY: return "VELOCITY";
  case rammp::ControlMode::POSITION: return "POSITION";
  case rammp::ControlMode::HOLD: return "HOLD";
  }
  return "?";
}
const char *fault_name(rammp::FaultCode f) {
  switch (f) {
  case rammp::FaultCode::NONE: return "none";
  case rammp::FaultCode::WATCHDOG: return "WATCHDOG";
  case rammp::FaultCode::OVERCURRENT: return "OVERCURRENT";
  case rammp::FaultCode::VDS_OCP: return "VDS_OCP";
  case rammp::FaultCode::OVERTEMP: return "OVERTEMP";
  case rammp::FaultCode::ENCODER: return "ENCODER";
  case rammp::FaultCode::DRV_FAULT: return "DRV_FAULT";
  case rammp::FaultCode::UNASSIGNED_AXIS: return "UNASSIGNED_AXIS";
  case rammp::FaultCode::SAMPLER: return "SAMPLER";
  case rammp::FaultCode::HALL: return "HALL";
  case rammp::FaultCode::STALL: return "STALL";
  }
  return "?";
}

double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void print_state(const rammp::MotorState &s, double t, uint32_t n) {
  printf("[%8.2f] #%-5u %-13s %-8s fault=%-15s seq=%3u last_cmd=%3u ack=%u age=%5ums | "
         "pos=%8.3f vel=%7.3f tq=%6.2f iq=%6.2f id=%6.2f vbus=%5.1f ibus=%5.2f | "
         "lim tq=%.1f vel=%.1f acc=%.1f | T=%.0f/%.0f/%.0f/%.0f drv=0x%04x\n",
         t, n, state_name(s.state), mode_name(s.mode), fault_name(s.fault_code), s.seq,
         s.last_cmd_seq, s.clear_fault_ack, s.cmd_age_ms, s.position, s.velocity, s.torque_est,
         s.iq, s.id, s.vbus, s.ibus_est, s.torque_limit_eff, s.vel_limit_eff, s.accel_limit_eff,
         s.temps[0], s.temps[1], s.temps[2], s.temps[3], s.drv_status);
  fflush(stdout);
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: mib_stub <interface-ip> <axis-segment> [seconds] [-v]\n");
    return 2;
  }
  const std::string iface = argv[1];
  const std::string seg = argv[2];
  double secs = 1e9;
  bool verbose = false;
  for (int i = 3; i < argc; i++) {
    if (strcmp(argv[i], "-v") == 0)
      verbose = true;
    else
      secs = atof(argv[i]);
  }

  const rammp::AxisSpec *axis = nullptr;
  for (const auto &a : rammp::kAxes)
    if (seg == a.segment)
      axis = &a;
  if (!axis) {
    fprintf(stderr, "unknown axis '%s'; one of:", seg.c_str());
    for (const auto &a : rammp::kAxes)
      fprintf(stderr, " %s", a.segment);
    fprintf(stderr, "\n");
    return 2;
  }

  std::atomic<uint32_t> pub_matched{0}, sub_matched{0}, rx{0}, tx_ok{0}, tx_fail{0};

  espp::RtpsParticipant::Config pcfg;
  pcfg.interface_address = iface;
  pcfg.on_publisher_matched = [&]() { pub_matched.fetch_add(1); };
  pcfg.on_subscriber_matched = [&]() { sub_matched.fetch_add(1); };
  // One receive port per reader, not a band of them: with dedicated endpoint
  // ports on, the board's writer sends every sample to each advertised
  // locator and the reader sees it three times (measured 2026-09-25).
  pcfg.enable_dedicated_endpoint_ports = false;
  pcfg.log_level = espp::Logger::Verbosity::WARN;
  espp::RtpsParticipant part(pcfg);
  if (!part.start()) {
    fprintf(stderr, "participant start FAILED on %s\n", iface.c_str());
    return 1;
  }

  std::mutex st_mutex;
  rammp::MotorState last{};
  bool have_state = false;
  double last_rx_t = 0;

  espp::Subscriber<rammp::MotorState> sub(
      part, {.topic = axis->state.name,
             .type_name = axis->state.type,
             .on_message =
                 [&](const rammp::MotorState &s) {
                   const uint32_t n = rx.fetch_add(1) + 1;
                   std::lock_guard<std::mutex> lk(st_mutex);
                   last = s;
                   have_state = true;
                   last_rx_t = now_s();
                   if (verbose)
                     print_state(s, last_rx_t, n);
                 }});
  espp::Publisher<rammp::MotorCommand> pub(
      part, {.topic = axis->command.name, .type_name = axis->command.type});
  if (!sub.is_valid() || !pub.is_valid()) {
    fprintf(stderr, "endpoint registration FAILED\n");
    return 1;
  }
  printf("mib_stub on %s -> axis %s\n  cmd   %s (%s)\n  state %s (%s)\n", iface.c_str(),
         axis->segment, axis->command.name, axis->command.type, axis->state.name,
         axis->state.type);
  fflush(stdout);

  // The command the 20 Hz stream sends. stdin edits it under the mutex.
  std::mutex cmd_mutex;
  rammp::MotorCommand cmd{};
  cmd.requested_state = rammp::RequestedState::DISARMED;
  cmd.mode = rammp::ControlMode::COAST;
  std::atomic<bool> run{true};

  std::thread streamer([&]() {
    auto next = std::chrono::steady_clock::now();
    while (run.load()) {
      {
        std::lock_guard<std::mutex> lk(cmd_mutex);
        cmd.seq++;
        if (pub.publish(cmd))
          tx_ok.fetch_add(1);
        else
          tx_fail.fetch_add(1);
      }
      next += std::chrono::milliseconds(50);
      std::this_thread::sleep_until(next);
    }
  });

  std::thread reporter([&]() {
    uint32_t shown = 0;
    while (run.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::lock_guard<std::mutex> lk(st_mutex);
      const uint32_t n = rx.load();
      if (have_state && n != shown && !verbose) {
        print_state(last, last_rx_t, n);
        shown = n;
      } else if (!have_state || n == shown) {
        printf("[%8.2f] no MotorState (rx=%u tx=%u/%u matched pub=%u sub=%u)\n", now_s(), n,
               tx_ok.load(), tx_fail.load(), pub_matched.load(), sub_matched.load());
        fflush(stdout);
      }
    }
  });

  const double t0 = now_s();
  char line[128];
  while (run.load() && now_s() - t0 < secs && fgets(line, sizeof line, stdin)) {
    float a = 0, b = 0, c = 0;
    std::lock_guard<std::mutex> lk(cmd_mutex);
    if (strncmp(line, "arm", 3) == 0) {
      cmd.requested_state = rammp::RequestedState::ARMED;
    } else if (strncmp(line, "disarm", 6) == 0) {
      cmd.requested_state = rammp::RequestedState::DISARMED;
    } else if (strncmp(line, "stop", 4) == 0) {
      cmd.requested_state = rammp::RequestedState::SAFE_STOP;
    } else if (strncmp(line, "estop", 5) == 0) {
      cmd.requested_state = rammp::RequestedState::ESTOP;
    } else if (strncmp(line, "coast", 5) == 0) {
      cmd.mode = rammp::ControlMode::COAST;
    } else if (strncmp(line, "hold", 4) == 0) {
      cmd.mode = rammp::ControlMode::HOLD;
    } else if (sscanf(line, "torque %f", &a) == 1) {
      cmd.mode = rammp::ControlMode::TORQUE;
      cmd.torque = a;
    } else if (sscanf(line, "vel %f", &a) == 1) {
      cmd.mode = rammp::ControlMode::VELOCITY;
      cmd.velocity = a;
    } else if (sscanf(line, "pos %f", &a) == 1) {
      cmd.mode = rammp::ControlMode::POSITION;
      cmd.position = a;
    } else if (sscanf(line, "lim %f %f %f", &a, &b, &c) == 3) {
      cmd.torque_limit = a;
      cmd.vel_limit = b;
      cmd.accel_limit = c;
    } else if (strncmp(line, "clr", 3) == 0) {
      cmd.clear_fault_req_id++;
    } else if (sscanf(line, "wait %f", &a) == 1) {
      cmd_mutex.unlock();
      std::this_thread::sleep_for(std::chrono::duration<double>(a));
      cmd_mutex.lock();
      continue;
    } else if (line[0] == 'q') {
      break;
    } else if (line[0] == '\n' || line[0] == '#') {
      continue;
    } else {
      printf("? %s", line);
      continue;
    }
    printf("> %s", line);
    fflush(stdout);
  }

  run.store(false);
  streamer.join();
  reporter.join();
  printf("\nrx=%u tx=%u/%u matched pub=%u sub=%u\n", rx.load(), tx_ok.load(), tx_fail.load(),
         pub_matched.load(), sub_matched.load());
  part.stop();
  return rx.load() > 0 ? 0 : 3;
}
