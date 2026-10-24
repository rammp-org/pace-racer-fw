#pragma once

// DemoLink: the native-USB (WebUSB) side of the dyno demo, shared verbatim by
// the brake bench and the PACE RACER firmware.
//
// One espp::UsbDevice on the ESP32-S3's USB-OTG peripheral with two functions:
//
//   vendor / WebUSB  one framed byte stream (espp stream_frame), multiplexed
//                    by an espp::DispatcherWorker into modules:
//                      0    OTA          espp::OtaService   (firmware update, restart, rollback)
//                      3    telemetry    espp::Telemetry    (typed float channels, device timestamps)
//                      4    core dump    espp::CoreDumpService (crash report, core.elf, erase)
//                      0x10 console      text commands in, stdout mirrored out
//                      0x20 system       reset, info
//                      0xFF discovery    what the device serves (espp Device Hub)
//   CDC-ACM          the plain console for `screen` / the espp Web Serial tools:
//                    stdout is mirrored here and typed lines feed the same
//                    command handler.
//
// Because USB-OTG and USB-Serial/JTAG share the S3's one USB PHY, the ROM
// console and esptool's DTR/RTS reset are gone once this is up. Reflashing is
// OTA over the vendor stream (`idf.py ota-usb`, or the dashboard's Update
// button); a reset is the system module's RESET request. A panic writes a core
// dump to flash (there is no live console during a panic) and the next boot
// serves it over module 4.
//
// stdout is redirected to a tiny VFS device whose write() forwards each chunk
// to (a) the primary UART console if the build has one, (b) the CDC interface,
// and (c) a console-module LOG frame on the vendor stream. All three are
// best-effort and non-blocking: logging must never stall a control loop.
//
// The USB sinks are fed by a drain task pinned to TinyUSB's own core, never by
// the printing task: a TinyUSB write runs its FIFO copy inside a spinlock
// critical section (dcd_edpt_xfer_fifo), which on the printing task's core
// masks every interrupt up to level 3 and spins against the TinyUSB task on
// the other. On the PACE RACER that core is the one with the PWM sampling ISR:
// a 3000-row capdump made 2.1 % of the samples late, each late sample skips
// a control update, and runs of them froze the duties for 220 us (2026-10-23).

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "coredump.hpp"
#include "coredump_service.hpp"
#include "dispatcher_worker.hpp"
#include "logger.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "stream_frame.hpp"
#include "telemetry.hpp"
#include "usb_device.hpp"

namespace demo {

/// Module ids beyond the espp defaults (0 OTA, 3 telemetry, 4 core dump, 0xFF discovery).
inline constexpr uint8_t kConsoleModule = 0x10;
inline constexpr uint8_t kSystemModule = 0x20;

/// Console module (0x10) message types.
inline constexpr uint8_t kConsoleLine = 0x01;   ///< host -> device: UTF-8 command line (no newline needed)
inline constexpr uint8_t kConsoleAck = 0x81;    ///< device -> host: line queued
inline constexpr uint8_t kConsoleLog = 0x82;    ///< device -> host: UTF-8 stdout chunk

/// System module (0x20) message types.
inline constexpr uint8_t kSystemReset = 0x01;   ///< host -> device: reboot (reply OK, then esp_restart)
inline constexpr uint8_t kSystemInfo = 0x02;    ///< host -> device: reply with a JSON info blob
inline constexpr uint8_t kSystemOk = 0x81;      ///< device -> host: reply to RESET ([u8 request type])
inline constexpr uint8_t kSystemInfoReply = 0x82; ///< device -> host: UTF-8 JSON

class DemoLink {
public:
  struct Config {
    std::string product;                 ///< USB product string; the dashboard routes on it.
    std::string serial{"000000000001"};  ///< USB serial string.
    uint16_t pid{0x0d32};                ///< USB PID (VID stays 0x1209, the espp default).
    std::vector<std::string> channels;   ///< Telemetry channel names, in emit() order.
    /// Called (on the USB worker task) right before an OTA session begins, so the
    /// application can put the plant in a safe state: flash writes disable the
    /// cache, which stalls anything running from flash for milliseconds at a time.
    std::function<void()> before_ota{nullptr};
    /// Seconds after boot at which a PENDING_VERIFY image confirms itself if no
    /// host has done so. 0 disables self-confirmation (pure host-driven rollback).
    /// A board that is only reachable over ethernet must leave this at 0 and let
    /// EthLink confirm once the network is actually up (see eth_link.hpp).
    uint32_t self_verify_after_s{30};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  explicit DemoLink(const Config &cfg)
      : cfg_(cfg)
      , logger_({.tag = "DemoLink", .level = cfg.log_level})
      , usb_(make_usb_config(cfg))
      , ota_({.reject_same_version = false, .log_level = espp::Logger::Verbosity::INFO})
      , ota_service_(ota_, {.send = [this](std::span<const uint8_t> f) { usb_.write_vendor(f); },
                            .log_level = espp::Logger::Verbosity::INFO})
      , core_dump_({.log_level = espp::Logger::Verbosity::WARN})
      , coredump_service_(core_dump_,
                          {.send = [this](std::span<const uint8_t> f) { usb_.write_vendor(f); }})
      , telemetry_({.channels = cfg.channels,
                    .send = [this](std::span<const uint8_t> f) { send_best_effort(f); },
                    // Quiet until a host asks (SET_STREAM): the espp_ota CLI reads
                    // the vendor endpoint expecting only its own replies, and an
                    // unsolicited sample stream overflows it. The dashboard turns
                    // streaming on when it connects and off when it leaves.
                    .stream_on_start = false,
                    .period_ms = 20})
      , link_({.send = [this](std::span<const uint8_t> f) { usb_.write_vendor(f); },
               .on_overflow = [this]() { ota_service_.on_rx_overflow(); },
               .task_config = {.name = "usb_rx", .stack_size_bytes = 8192}}) {
    s_instance = this;
  }

  ~DemoLink() { s_instance = nullptr; }

  DemoLink(const DemoLink &) = delete;
  DemoLink &operator=(const DemoLink &) = delete;

  /// Bring the USB device up, route stdout, register every module. Returns
  /// false (with ec) if the USB device could not initialize; the console
  /// then stays wherever the build put it.
  bool start(std::error_code &ec) {
    // OTA (module 0): wrap the service so the application's before_ota runs
    // on BEGIN. The service still owns the whole session state machine.
    link_.register_module(
        ota_service_.module_id(),
        [this](const espp::stream_frame::Frame &f) {
          if (!f.is_reply() && f.type == 0x01 /* BEGIN */ && cfg_.before_ota) {
            logger_.warn("OTA begin: putting the plant in a safe state");
            cfg_.before_ota();
          }
          ota_service_.handle(f);
        },
        ota_service_.module_info());
    link_.register_module(telemetry_);
    link_.register_module(coredump_service_);
    link_.register_module(
        kConsoleModule, [this](const espp::stream_frame::Frame &f) { handle_console(f); },
        {.name = "Console", .app = "", .description = "Text commands in, stdout out"});
    link_.register_module(
        kSystemModule, [this](const espp::stream_frame::Frame &f) { handle_system(f); },
        {.name = "System", .app = "", .description = "Reset and device info"});
    link_.serve_discovery(cfg_.product, ota_.running_app_description().version);

    usb_.set_vendor_receive_callback([this](std::span<const uint8_t> d) { link_.push(d); });
    usb_.set_cdc_receive_callback([this](std::span<const uint8_t> d) {
      // A terminal types characters; feed them straight into the line queue.
      push_bytes(std::string_view(reinterpret_cast<const char *>(d.data()), d.size()));
    });

    if (!usb_.initialize(ec)) {
      logger_.error("USB device failed to initialize: {}", ec.message());
      return false;
    }
    route_stdout();

    const auto app = ota_.running_app_description();
    logger_.info("USB up as '{}' | fw {} {} ({} {}) on {}{}", cfg_.product, app.project_name,
                 app.version, app.date, app.time, ota_.running_partition_label(),
                 ota_.is_pending_verify() ? " | PENDING VERIFY" : "");
    if (const auto report = core_dump_.format_report(); !report.empty())
      fmt::print("\n=== previous reset ===\n{}\n", report);

    if (ota_.is_pending_verify() && cfg_.self_verify_after_s > 0)
      start_self_verify_timer();
    return true;
  }

  /// One telemetry sample, values in Config::channels order.
  void emit(std::span<const float> values) { telemetry_.emit(values); }

  /// Next queued console byte (from CDC or a console-module LINE frame), or -1.
  int getc() {
    std::lock_guard<std::mutex> lk(rx_mutex_);
    if (rx_.empty())
      return -1;
    const int c = static_cast<unsigned char>(rx_.front());
    rx_.pop_front();
    return c;
  }

  /// Queue a console line from another transport (the ethernet link) as if
  /// it had been typed on CDC: the app's console loop reads it with getc().
  void push_line(std::string_view line) { push_bytes(line); }
  /// Extra stdout sink (the ethernet link's LOG frames). Called from inside
  /// stdout writes on whatever task is printing: it must never block.
  void set_log_sink(std::function<void(std::span<const uint8_t>)> sink) {
    std::lock_guard<std::mutex> lk(sink_mutex_);
    log_sink_ = std::move(sink);
  }

  /// stdout sinks to feed, for A/B timing work: bit 0 UART, 1 CDC, 2 vendor
  /// LOG frames, 3 the extra sink (ethernet). Default all. Bit 4 = write the
  /// USB sinks inline from the printing task (the pre-drain behaviour).
  static void set_stdout_sinks(uint8_t mask) { s_sinks.store(mask, std::memory_order_relaxed); }
  static uint8_t stdout_sinks() { return s_sinks.load(std::memory_order_relaxed); }

  bool vendor_connected() const { return usb_.is_vendor_connected(); }
  /// The vendor-stream dispatcher, for application modules (register before or
  /// after start(); handlers run on the USB worker task).
  espp::DispatcherWorker &worker() { return link_; }
  /// Reply on the vendor stream (blocking, bounded): for request/reply modules.
  void reply(std::span<const uint8_t> frame) { usb_.write_vendor(frame); }
  espp::Ota &ota() { return ota_; }
  espp::UsbDevice &usb() { return usb_; }

private:
  static espp::UsbDevice::Config make_usb_config(const Config &cfg) {
    espp::UsbDevice::Config u;
    u.pid = cfg.pid;
    u.manufacturer = "RAMMP / AT Dev";
    u.product = cfg.product;
    u.serial_number = cfg.serial;
    u.log_level = espp::Logger::Verbosity::WARN;
    espp::UsbDevice::VendorFunction vendor;
    vendor.interface_name = cfg.product + " (WebUSB)";
    vendor.webusb = true;
    vendor.landing_page_url = "esp-cpp.github.io/espp/apps/dispatcher_hub.html";
    u.vendor = vendor;
    espp::UsbDevice::CdcFunction cdc;
    cdc.interface_name = cfg.product + " console";
    cdc.route_console = false; // we own stdout (see route_stdout), so it reaches the vendor log too
    u.cdc = cdc;
    return u;
  }

  /// Telemetry and log frames are fire-and-forget. write_vendor() bounded-waits
  /// up to 250 ms for FIFO room, which with no host reading the vendor endpoint
  /// would turn every sample into a 250 ms stall of the producing loop (seen:
  /// the bench's 70 Hz poll fell to 4 Hz). So: room now, or drop silently.
  void send_best_effort(std::span<const uint8_t> f) {
    if (usb_.is_vendor_connected() && usb_.vendor_write_available() >= f.size())
      usb_.write_vendor(f);
  }

  // ---- console module -------------------------------------------------------
  void push_bytes(std::string_view s) {
    std::lock_guard<std::mutex> lk(rx_mutex_);
    for (char c : s) {
      if (rx_.size() >= kRxCap)
        rx_.pop_front();
      rx_.push_back(c);
    }
  }

  void handle_console(const espp::stream_frame::Frame &f) {
    if (f.is_reply() || f.type != kConsoleLine)
      return;
    std::string line(reinterpret_cast<const char *>(f.payload.data()), f.payload.size());
    line.push_back('\n');
    push_bytes(line);
    usb_.write_vendor(espp::stream_frame::build_frame(true, kConsoleModule, kConsoleAck));
  }

  // ---- system module --------------------------------------------------------
  void handle_system(const espp::stream_frame::Frame &f) {
    if (f.is_reply())
      return;
    if (f.type == kSystemReset) {
      const uint8_t which = kSystemReset;
      usb_.write_vendor(espp::stream_frame::build_frame(true, kSystemModule, kSystemOk,
                                                        std::span<const uint8_t>(&which, 1)));
      logger_.warn("reset requested over USB");
      std::thread([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        esp_restart();
      }).detach();
    } else if (f.type == kSystemInfo) {
      const auto app = ota_.running_app_description();
      const std::string json = fmt::format(
          "{{\"product\":\"{}\",\"project\":\"{}\",\"version\":\"{}\",\"built\":\"{} {}\","
          "\"partition\":\"{}\",\"pending_verify\":{},\"reset_reason\":\"{}\","
          "\"core_dump\":{},\"uptime_s\":{}}}",
          cfg_.product, app.project_name, app.version, app.date, app.time,
          ota_.running_partition_label(), ota_.is_pending_verify() ? "true" : "false",
          espp::CoreDump::reset_reason_name(espp::CoreDump::reset_reason()),
          core_dump_.has_core_dump() ? "true" : "false", esp_timer_get_time() / 1000000);
      usb_.write_vendor(espp::stream_frame::build_frame(
          true, kSystemModule, kSystemInfoReply,
          std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(json.data()), json.size())));
    }
  }

  // ---- rollback -------------------------------------------------------------
  void start_self_verify_timer() {
    std::thread([this] {
      std::this_thread::sleep_for(std::chrono::seconds(cfg_.self_verify_after_s));
      if (!ota_.is_pending_verify())
        return; // a host already decided
      std::error_code ec;
      if (ota_.mark_app_valid(ec))
        logger_.info("image self-confirmed after {} s of healthy running", cfg_.self_verify_after_s);
      else
        logger_.error("self-confirm failed: {}", ec.message());
    }).detach();
  }

  // ---- stdout tee -----------------------------------------------------------
  static constexpr char kVfsPath[] = "/dev/democon";
  static constexpr size_t kRxCap = 2048;
  static inline DemoLink *s_instance{nullptr};
  static inline int s_uart_fd{-1};
  static inline std::atomic<uint8_t> s_sinks{0x0f};
  // stdout bytes on their way to the USB sinks (see the header comment).
  static constexpr size_t kTxBufBytes = 8192;
  static inline StreamBufferHandle_t s_tx{nullptr};
  static inline std::mutex s_tx_mutex; // stream buffers take one writer at a time
  static inline TaskHandle_t s_tx_task{nullptr};
  static inline thread_local bool s_in_write{false};

  static int vfs_open(const char *, int, int) { return 0; }
  static int vfs_close(int) { return 0; }
  static int vfs_fstat(int, struct stat *st) {
    *st = {};
    st->st_mode = S_IFCHR;
    return 0;
  }
  static ssize_t vfs_write(int, const void *data, size_t size) {
    // A logging call made *by* the USB stack from inside a write would recurse
    // back into it; break the loop per task.
    bool &in_write = s_in_write;
    const uint8_t sinks = s_sinks.load(std::memory_order_relaxed);
    if (s_uart_fd >= 0 && (sinks & 1))
      ::write(s_uart_fd, data, size);
    if (in_write)
      return static_cast<ssize_t>(size);
    in_write = true;
    if (auto *self = s_instance) {
      const auto bytes = std::span<const uint8_t>(static_cast<const uint8_t *>(data), size);
      if (s_tx && !(sinks & 0x10)) {
        // Bounded wait for room (a bulk dump outruns USB only with the UART
        // tee off), then drop: never block a printer indefinitely.
        std::lock_guard<std::mutex> lk(s_tx_mutex);
        xStreamBufferSend(s_tx, data, size, pdMS_TO_TICKS(20));
      } else {
        self->usb_sinks(bytes, sinks);
      }
      if ((sinks & 8) && self->sink_mutex_.try_lock()) {
        if (self->log_sink_)
          self->log_sink_(bytes);
        self->sink_mutex_.unlock();
      }
    }
    in_write = false;
    return static_cast<ssize_t>(size);
  }

  void usb_sinks(std::span<const uint8_t> bytes, uint8_t sinks) {
    if ((sinks & 2) && usb_.is_cdc_connected() && usb_.cdc_write_available() >= bytes.size())
      usb_.write_cdc(bytes);
    // Only while a dashboard session is streaming telemetry (same reason as
    // stream_on_start = false). 13 = frame overhead (9-byte header + CRC-32).
    if ((sinks & 4) && telemetry_.streaming() && usb_.is_vendor_connected() &&
        usb_.vendor_write_available() >= bytes.size() + 13)
      usb_.write_vendor(
          espp::stream_frame::build_frame(true, kConsoleModule, kConsoleLog, bytes));
  }

  // Drains s_tx into the USB sinks, on TinyUSB's core. Waits (bounded) for CDC
  // room instead of dropping: the printer is already decoupled.
  static void tx_task(void *) {
    s_in_write = true; // whatever the USB stack logs from here goes to the UART only
    static uint8_t chunk[512];
    for (;;) {
      const size_t n = xStreamBufferReceive(s_tx, chunk, sizeof(chunk), portMAX_DELAY);
      auto *self = s_instance;
      if (!self || n == 0)
        continue;
      const uint8_t sinks = s_sinks.load(std::memory_order_relaxed);
      auto &usb = self->usb_;
      for (int i = 0; i < 50 && (sinks & 2) && usb.is_cdc_connected() &&
                      usb.cdc_write_available() < n;
           i++)
        vTaskDelay(1);
      self->usb_sinks(std::span<const uint8_t>(chunk, n), sinks);
    }
  }

  void start_tx_task() {
    TaskHandle_t tusb = xTaskGetHandle("TinyUSB");
    const BaseType_t core = tusb ? xTaskGetCoreID(tusb) : tskNO_AFFINITY;
    s_tx = xStreamBufferCreate(kTxBufBytes, 1);
    if (!s_tx)
      return;
    // Below TinyUSB (5): it has to run to drain the FIFO this task fills.
    if (xTaskCreatePinnedToCore(tx_task, "con_tx", 4096, nullptr, 3, &s_tx_task, core) != pdPASS) {
      vStreamBufferDelete(s_tx);
      s_tx = nullptr;
      return;
    }
    logger_.info("stdout -> USB drained on core {}", (int)core);
  }

  void route_stdout() {
    start_tx_task();
#if defined(CONFIG_ESP_CONSOLE_UART_NUM) && CONFIG_ESP_CONSOLE_UART_NUM >= 0
    char uart[16];
    std::snprintf(uart, sizeof(uart), "/dev/uart/%d", CONFIG_ESP_CONSOLE_UART_NUM);
    s_uart_fd = ::open(uart, O_WRONLY);
#endif
    esp_vfs_t vfs = {};
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    vfs.open = &vfs_open;
    vfs.write = &vfs_write;
    vfs.close = &vfs_close;
    vfs.fstat = &vfs_fstat;
#pragma GCC diagnostic pop
    if (esp_vfs_register(kVfsPath, &vfs, nullptr) != ESP_OK) {
      logger_.error("could not register the console VFS; stdout stays put");
      return;
    }
    fflush(stdout);
    if (freopen(kVfsPath, "w", stdout) == nullptr) {
      esp_vfs_unregister(kVfsPath);
      return;
    }
    setvbuf(stdout, nullptr, _IONBF, 0);
  }

  Config cfg_;
  espp::Logger logger_;
  espp::UsbDevice usb_;
  espp::Ota ota_;
  espp::OtaService ota_service_;
  espp::CoreDump core_dump_;
  espp::CoreDumpService coredump_service_;
  espp::Telemetry telemetry_;
  espp::DispatcherWorker link_;

  std::mutex rx_mutex_;
  std::deque<char> rx_;
  std::mutex sink_mutex_;
  std::function<void(std::span<const uint8_t>)> log_sink_;
};

} // namespace demo
