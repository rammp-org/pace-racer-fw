#pragma once

// EthLink: the same framed service stream DemoLink serves over the WebUSB
// vendor endpoint, carried over a TCP socket on the board's ethernet. Once a
// board is sealed inside a wheel its USB port is gone; this is how it is
// flashed, provisioned and reset from then on.
//
//   TCP :5555   one espp stream_frame byte stream, one client at a time,
//               multiplexed by its own espp::DispatcherWorker into modules:
//                 0    OTA       espp::OtaService (second instance on the SAME
//                                espp::Ota engine as USB; each instance only
//                                owns the session it began)
//                 0x10 console   LINE in -> the app's console queue, ACK back;
//                                stdout mirrored out as LOG frames while a
//                                client is connected
//                 0x20 system    RESET, INFO (same wire format as USB)
//                 0xFF discovery
//   UDP :5556   "PACE?" in (broadcast or unicast) -> one JSON line back,
//               unicast to the asker: who this board is and where to connect.
//               Host-initiated on purpose: the bench switch eats board-
//               originated multicast (see atos-bench-network), but a unicast
//               reply to a broadcast question always gets through.
//
// Rollback gate. A new image boots PENDING_VERIFY. The USB link used to
// self-confirm 30 s after boot; for a board that can only be reached over
// ethernet that is backwards: an image whose ethernet is broken would confirm
// itself and strand the wheel. Here the image self-confirms only once it has
// a lease AND this server is accepting, after Config::self_verify_after_s. An
// image that never gets that far stays pending and the bootloader falls back
// to the previous image at the next reset (power cycle). The host tool also
// confirms explicitly once it reconnects, which is the normal path.
//
// All socket work lives on one task: a select() loop over the listen socket,
// the client and the UDP responder. The dispatcher worker's send and the
// stdout tee write to the client socket through one mutex; the stdout path is
// non-blocking (queue or drop), logging must never stall a control loop.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "dispatcher_worker.hpp"
#include "logger.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "stream_frame.hpp"
#include "task.hpp"

#include "demo_link.hpp" // module ids and message types, shared with USB

namespace demo {

class EthLink {
public:
  struct Config {
    uint16_t port{5555};           ///< TCP service port
    uint16_t discovery_port{5556}; ///< UDP "PACE?" responder; 0 disables
    std::string product;           ///< discovery device name
    /// Called right before an OTA session begins (same contract as DemoLink).
    std::function<void()> before_ota{nullptr};
    /// Console lines from the client go here (the app's console queue).
    std::function<void(std::string_view)> on_line{nullptr};
    /// JSON object for INFO and the discovery reply: the app adds axis,
    /// calibration state and anything else a host wants to route on.
    std::function<std::string()> info_json{nullptr};
    /// The current IPv4 lease, empty when there is none.
    std::function<std::string()> ip_fn{nullptr};
    /// Seconds of (lease + server accepting) after which a PENDING_VERIFY
    /// image confirms itself. 0 = never (pure host-driven).
    uint32_t self_verify_after_s{20};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  EthLink(const Config &cfg, espp::Ota &ota)
      : cfg_(cfg)
      , logger_({.tag = "EthLink", .level = cfg.log_level})
      , ota_(ota)
      , ota_service_(ota, {.send = [this](std::span<const uint8_t> f) { send(f); },
                           .log_level = espp::Logger::Verbosity::INFO})
      , worker_({.send = [this](std::span<const uint8_t> f) { send(f); },
                 .on_overflow = [this]() { ota_service_.on_rx_overflow(); },
                 .task_config = {.name = "eth_rx", .stack_size_bytes = 8192}}) {
    worker_.register_module(
        ota_service_.module_id(),
        [this](const espp::stream_frame::Frame &f) {
          if (!f.is_reply() && f.type == 0x01 /* BEGIN */ && cfg_.before_ota) {
            logger_.warn("OTA begin over ethernet: putting the plant in a safe state");
            cfg_.before_ota();
          }
          ota_service_.handle(f);
        },
        ota_service_.module_info());
    worker_.register_module(
        kConsoleModule, [this](const espp::stream_frame::Frame &f) { handle_console(f); },
        {.name = "Console", .app = "", .description = "Text commands in, stdout out"});
    worker_.register_module(
        kSystemModule, [this](const espp::stream_frame::Frame &f) { handle_system(f); },
        {.name = "System", .app = "", .description = "Reset and device info"});
    worker_.serve_discovery(cfg_.product, ota_.running_app_description().version);
  }

  ~EthLink() { stop(); }
  EthLink(const EthLink &) = delete;
  EthLink &operator=(const EthLink &) = delete;

  /// Open the sockets and start the server task. Needs the TCP/IP stack up
  /// (the BSP's init_ethernet); a lease is not needed, 0.0.0.0 listens fine.
  bool start() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd_ < 0) {
      logger_.error("socket() failed: {}", errno);
      return false;
    }
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(cfg_.port);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
        ::listen(listen_fd_, 1) < 0) {
      logger_.error("bind/listen on :{} failed: {}", cfg_.port, errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    if (cfg_.discovery_port) {
      udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (udp_fd_ >= 0) {
        ::setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in u{};
        u.sin_family = AF_INET;
        u.sin_addr.s_addr = htonl(INADDR_ANY);
        u.sin_port = htons(cfg_.discovery_port);
        if (::bind(udp_fd_, reinterpret_cast<sockaddr *>(&u), sizeof(u)) < 0) {
          logger_.error("discovery bind on :{} failed: {}", cfg_.discovery_port, errno);
          ::close(udp_fd_);
          udp_fd_ = -1;
        }
      }
    }
    running_.store(true);
    task_ = espp::Task::make_unique(espp::Task::Config{
        .callback = [this](std::mutex &m, std::condition_variable &cv) { return serve(m, cv); },
        .task_config = {.name = "eth_link", .stack_size_bytes = 6 * 1024, .priority = 4, .core_id = -1},
        .log_level = espp::Logger::Verbosity::WARN,
    });
    task_->start();
    logger_.info("service stream on tcp :{}{}", cfg_.port,
                 udp_fd_ >= 0 ? fmt::format(", discovery on udp :{}", cfg_.discovery_port) : "");
    if (ota_.is_pending_verify() && cfg_.self_verify_after_s > 0)
      start_self_verify();
    return true;
  }

  void stop() {
    running_.store(false);
    if (task_) {
      task_->stop();
      task_.reset();
    }
    close_client();
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (udp_fd_ >= 0) {
      ::close(udp_fd_);
      udp_fd_ = -1;
    }
  }

  bool client_connected() const { return client_fd_.load() >= 0; }

  /// stdout tee: queue a chunk for the connected client as a LOG frame. Never
  /// blocks; with no client, or a full queue, the chunk is dropped.
  void log_sink(std::span<const uint8_t> bytes) {
    if (!client_connected() || bytes.empty())
      return;
    auto frame = espp::stream_frame::build_frame(true, kConsoleModule, kConsoleLog, bytes);
    std::lock_guard<std::mutex> lk(log_mutex_);
    if (log_bytes_ + frame.size() > kLogCap) {
      log_dropped_++;
      return;
    }
    log_bytes_ += frame.size();
    log_q_.push_back(std::move(frame));
  }

  espp::DispatcherWorker &worker() { return worker_; }

private:
  static constexpr size_t kLogCap = 16 * 1024;
  static constexpr size_t kRxChunk = 2048;

  // ---- server task -----------------------------------------------------------
  bool serve(std::mutex &, std::condition_variable &) {
    if (!running_.load())
      return true;
    fd_set rfds;
    FD_ZERO(&rfds);
    int maxfd = -1;
    auto add = [&](int fd) {
      if (fd >= 0) {
        FD_SET(fd, &rfds);
        maxfd = std::max(maxfd, fd);
      }
    };
    add(listen_fd_);
    add(udp_fd_);
    const int client = client_fd_.load();
    add(client);
    timeval tv{.tv_sec = 0, .tv_usec = 50 * 1000};
    const int n = ::select(maxfd + 1, &rfds, nullptr, nullptr, &tv);
    if (n > 0) {
      if (listen_fd_ >= 0 && FD_ISSET(listen_fd_, &rfds))
        accept_client();
      if (udp_fd_ >= 0 && FD_ISSET(udp_fd_, &rfds))
        answer_discovery();
      if (client >= 0 && FD_ISSET(client, &rfds))
        read_client(client);
    }
    flush_log();
    return false;
  }

  void accept_client() {
    sockaddr_in peer{};
    socklen_t len = sizeof(peer);
    const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr *>(&peer), &len);
    if (fd < 0)
      return;
    // One client; a newcomer replaces a stale session (a tool that crashed
    // mid-OTA, a laptop that went to sleep). Any OTA that session began is
    // left to the engine's own timeout.
    close_client();
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    timeval snd{.tv_sec = 2, .tv_usec = 0};
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
    char ip[16] = "?";
    inet_ntoa_r(peer.sin_addr, ip, sizeof(ip));
    worker_.request_reset();
    parser_bytes_ = 0;
    client_fd_.store(fd);
    logger_.info("client {}:{} connected", ip, ntohs(peer.sin_port));
  }

  void read_client(int fd) {
    uint8_t buf[kRxChunk];
    const int r = ::recv(fd, buf, sizeof(buf), 0);
    if (r <= 0) {
      logger_.info("client gone ({})", r == 0 ? "closed" : "error");
      close_client();
      return;
    }
    parser_bytes_ += r;
    worker_.push(std::span<const uint8_t>(buf, static_cast<size_t>(r)));
  }

  void close_client() {
    const int fd = client_fd_.exchange(-1);
    if (fd >= 0) {
      std::lock_guard<std::mutex> lk(send_mutex_);
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
    std::lock_guard<std::mutex> lk(log_mutex_);
    log_q_.clear();
    log_bytes_ = 0;
  }

  void answer_discovery() {
    char buf[64];
    sockaddr_in from{};
    socklen_t len = sizeof(from);
    const int r = ::recvfrom(udp_fd_, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr *>(&from), &len);
    if (r <= 0)
      return;
    buf[r] = 0;
    if (std::strncmp(buf, "PACE?", 5) != 0)
      return;
    const std::string reply = info();
    ::sendto(udp_fd_, reply.data(), reply.size(), 0, reinterpret_cast<sockaddr *>(&from), len);
  }

  // ---- sending ---------------------------------------------------------------
  /// Blocking (bounded by SO_SNDTIMEO) write of one frame; dispatcher replies.
  void send(std::span<const uint8_t> f) {
    const int fd = client_fd_.load();
    if (fd < 0)
      return;
    std::lock_guard<std::mutex> lk(send_mutex_);
    size_t off = 0;
    while (off < f.size()) {
      const int w = ::send(fd, f.data() + off, f.size() - off, 0);
      if (w <= 0) {
        logger_.warn("send failed ({}), dropping client", errno);
        // Can't close under send_mutex_ (close_client takes it); flag it.
        client_lost_.store(true);
        return;
      }
      off += static_cast<size_t>(w);
    }
  }

  void flush_log() {
    if (client_lost_.exchange(false))
      close_client();
    std::deque<std::vector<uint8_t>> batch;
    {
      std::lock_guard<std::mutex> lk(log_mutex_);
      if (log_q_.empty())
        return;
      batch.swap(log_q_);
      log_bytes_ = 0;
    }
    for (const auto &f : batch)
      send(f);
  }

  // ---- modules ---------------------------------------------------------------
  void handle_console(const espp::stream_frame::Frame &f) {
    if (f.is_reply() || f.type != kConsoleLine)
      return;
    std::string line(reinterpret_cast<const char *>(f.payload.data()), f.payload.size());
    line.push_back('\n');
    if (cfg_.on_line)
      cfg_.on_line(line);
    send(espp::stream_frame::build_frame(true, kConsoleModule, kConsoleAck));
  }

  void handle_system(const espp::stream_frame::Frame &f) {
    if (f.is_reply())
      return;
    if (f.type == kSystemReset) {
      const uint8_t which = kSystemReset;
      send(espp::stream_frame::build_frame(true, kSystemModule, kSystemOk,
                                           std::span<const uint8_t>(&which, 1)));
      logger_.warn("reset requested over ethernet");
      std::thread([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        esp_restart();
      }).detach();
    } else if (f.type == kSystemInfo) {
      const std::string json = info();
      send(espp::stream_frame::build_frame(
          true, kSystemModule, kSystemInfoReply,
          std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(json.data()), json.size())));
    }
  }

  std::string info() const {
    const auto app = ota_.running_app_description();
    std::string extra = cfg_.info_json ? cfg_.info_json() : std::string{};
    // Base (eFuse) MAC: what esptool read-mac prints and provision.py's roster
    // keys on. The ethernet MAC is base + 3 on this chip.
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_BASE);
    return fmt::format("{{\"product\":\"{}\",\"project\":\"{}\",\"version\":\"{}\",\"built\":\"{} {}\","
                       "\"partition\":\"{}\",\"pending_verify\":{},\"ip\":\"{}\",\"port\":{},"
                       "\"mac\":\"{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\",\"uptime_s\":{}{}{}}}",
                       cfg_.product, app.project_name, app.version, app.date, app.time,
                       ota_.running_partition_label(), ota_.is_pending_verify() ? "true" : "false",
                       cfg_.ip_fn ? cfg_.ip_fn() : "", cfg_.port, mac[0], mac[1], mac[2], mac[3],
                       mac[4], mac[5], esp_timer_get_time() / 1000000, extra.empty() ? "" : ",",
                       extra);
  }

  // ---- rollback gate ---------------------------------------------------------
  void start_self_verify() {
    std::thread([this] {
      // Wait for a lease, however long that takes, then the healthy window.
      while (running_.load()) {
        if (!ota_.is_pending_verify())
          return; // a host decided
        if (cfg_.ip_fn && !cfg_.ip_fn().empty())
          break;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      std::this_thread::sleep_for(std::chrono::seconds(cfg_.self_verify_after_s));
      if (!running_.load() || !ota_.is_pending_verify())
        return;
      if (cfg_.ip_fn && cfg_.ip_fn().empty())
        return; // lease gone again; stay pending
      std::error_code ec;
      if (ota_.mark_app_valid(ec))
        logger_.info("image self-confirmed: lease held and service stream up for {} s",
                     cfg_.self_verify_after_s);
      else
        logger_.error("self-confirm failed: {}", ec.message());
    }).detach();
  }

  Config cfg_;
  espp::Logger logger_;
  espp::Ota &ota_;
  espp::OtaService ota_service_;
  espp::DispatcherWorker worker_;
  std::unique_ptr<espp::Task> task_;

  std::atomic<bool> running_{false};
  std::atomic<bool> client_lost_{false};
  int listen_fd_{-1};
  int udp_fd_{-1};
  std::atomic<int> client_fd_{-1};
  size_t parser_bytes_{0};

  std::mutex send_mutex_;
  std::mutex log_mutex_;
  std::deque<std::vector<uint8_t>> log_q_;
  size_t log_bytes_{0};
  uint32_t log_dropped_{0};
};

} // namespace demo
