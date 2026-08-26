// Host-side DDS subscriber for the pace-racer telemetry topic.
//
// Unlike tests/rtps_sub.py this is a real RTPS participant: it announces itself
// via SPDP and declares a reader via SEDP, so the board's writer actually matches
// it and transmits. Sample is copied verbatim from main/rtps_telem.hpp so the
// reflection-driven CDR layout is identical on both ends.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"

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

static std::atomic<uint32_t> g_count{0};
static std::atomic<uint32_t> g_matched{0};

int main(int argc, char **argv) {
  std::string iface = (argc > 1) ? argv[1] : "192.168.50.1";
  double secs = (argc > 2) ? atof(argv[2]) : 12.0;
  int show = (argc > 3) ? atoi(argv[3]) : 5;

  printf("interface=%s  duration=%.0fs\n", iface.c_str(), secs);
  printf("topic=pace_racer/telemetry  type=pace_racer::msg::Telemetry\n\n");

  espp::RtpsParticipant::Config pcfg;
  pcfg.interface_address = iface;
  pcfg.on_subscriber_matched = []() { g_matched.fetch_add(1); };
  pcfg.log_level = espp::Logger::Verbosity::DEBUG;

  espp::RtpsParticipant part(pcfg);
  if (!part.start()) {
    fprintf(stderr, "participant start FAILED\n");
    return 1;
  }

  auto t0 = std::chrono::steady_clock::now();
  std::atomic<double> first_t{-1.0}, last_t{-1.0};

  espp::Subscriber<Sample> sub(part, {
    .topic = "pace_racer/telemetry",
    .type_name = "pace_racer::msg::Telemetry",
    .on_message = [&](const Sample &s) {
      uint32_t n = g_count.fetch_add(1);
      if (first_t.load() < 0) first_t.store(s.seconds);
      last_t.store(s.seconds);
      if (show == -1) {
        // machine-readable: one line per sample, board timestamp + host arrival
        // absolute wall-clock epoch seconds: lets the Python driver align its
        // own rate-change timestamps to sample arrivals with no clock offset.
        printf("S %.4f %.6f\n", s.seconds,
               std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch()).count());
        fflush(stdout);
      } else if ((int)n < show) {
        printf("  t=%8.2f state=%c id=%6.2f iq=%6.2f iqref=%5.2f vd=%6.2f vq=%6.2f "
               "aerr=%7.1f rpm_drive=%6.1f rpm_est=%6.1f rpm_hall=%6.1f flux=%.4f "
               "temps=%.1f/%.1f/%.1f/%.1f\n",
               s.seconds, s.state ? s.state : '?', s.id, s.iq, s.iqref, s.vd, s.vq,
               s.aerr, s.rpm_drive, s.rpm_est, s.rpm_hall, s.flux,
               s.temps[0], s.temps[1], s.temps[2], s.temps[3]);
      }
    },
  });
  if (!sub.is_valid()) {
    fprintf(stderr, "reader registration FAILED\n");
    return 1;
  }
  printf("reader registered; waiting for the board's writer...\n");

  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  uint32_t n = g_count.load();
  printf("\nmatched callbacks : %u\n", g_matched.load());
  printf("samples received  : %u in %.2fs -> %.1f Hz\n", n, el, n / el);
  if (n > 1 && last_t.load() > first_t.load()) {
    printf("board clock span  : %.2fs -> %.1f Hz by board timestamps\n",
           last_t.load() - first_t.load(), (n - 1) / (last_t.load() - first_t.load()));
  }
  part.stop();
  return n > 0 ? 0 : 2;
}
