#pragma once

// Per-board provisioning in NVS: which wheel this board is, and the
// calibration the drive needs before it may move.
//
//   axis_id  rammp::AxisId. Factory default UNASSIGNED (255): the board joins no
//            topic and reports itself inert on the console until 'axis <n>'.
//   enc_ofs  encoder -> electrical offset, rad ('ecal' result). NaN = unknown;
//            the supervisor refuses to arm without it, because driving on a
//            wrong offset is the detent-lock mistake from the hall bring-up.
//   hall     the six hall sector angles, deg ('hcal' result / 'hset'). NaN =
//            uncalibrated. Restored at boot so a chair never needs 'hcal'.
//   fb       which sensor commutates and measures the shaft: HALL (default,
//            the chair has no encoder) or ENCODER (the dyno). Arming requires
//            the selected source to be calibrated; the other one is ignored.
//   spd_kp, spd_ki, pll_hz
//            speed-loop gains (A/rpm, A/rpm-s) and hall tracking-loop
//            bandwidth (Hz) saved by a MotorTuning with `persist`, or 'tune
//            save' on the console. NaN = not saved: the compiled defaults for
//            the feedback source apply. Restored at boot over the defaults.
//
// One NVS namespace, one key per field, all plain reads and writes; nothing
// here is timing critical. Saves happen from the console and stream tasks,
// never from the supervisor tick (a flash write is milliseconds).

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <nvs.h>
#include <nvs_flash.h>

#include "logger.hpp"
#include "rtps_interface.hpp"

namespace axiscfg {

inline constexpr const char *kNamespace = "atos";
inline constexpr const char *kKeyAxis = "axis_id";
inline constexpr const char *kKeyEncOfs = "enc_ofs";
inline constexpr const char *kKeyHall = "hall_deg";
inline constexpr const char *kKeyFeedback = "fb";
inline constexpr const char *kKeySpdKp = "spd_kp";
inline constexpr const char *kKeySpdKi = "spd_ki";
inline constexpr const char *kKeyPllHz = "pll_hz";

enum class Feedback : uint8_t { HALL = 0, ENCODER = 1 };
inline const char *feedback_name(Feedback f) { return f == Feedback::ENCODER ? "enc" : "hall"; }

struct AxisConfig {
  rammp::AxisId axis_id{rammp::AxisId::UNASSIGNED};
  float enc_ofs_rad{NAN};
  std::array<float, 6> hall_deg{NAN, NAN, NAN, NAN, NAN, NAN};
  Feedback feedback{Feedback::HALL};
  float spd_kp{NAN}, spd_ki{NAN}; ///< A/rpm, A/rpm-s; NaN = compiled default
  float pll_hz{NAN};              ///< Hz; NaN = compiled default

  bool tuning_saved() const { return !std::isnan(spd_kp) && !std::isnan(spd_ki); }
  bool assigned() const { return axis_id != rammp::AxisId::UNASSIGNED; }
  bool enc_calibrated() const { return !std::isnan(enc_ofs_rad); }
  bool hall_calibrated() const {
    for (float d : hall_deg)
      if (std::isnan(d))
        return false;
    return true;
  }
  /// The source the drive will commutate on is calibrated.
  bool feedback_calibrated() const {
    return feedback == Feedback::ENCODER ? enc_calibrated() : hall_calibrated();
  }
};

/// True if `id` is a row of RAMMP_AXIS_TABLE.
inline bool valid_axis(uint8_t id) {
  for (const auto &a : rammp::kAxes)
    if (static_cast<uint8_t>(a.id) == id)
      return true;
  return false;
}

/// "drive_left" for a table id, "unassigned" otherwise.
inline const char *axis_name(rammp::AxisId id) {
  if (!valid_axis(static_cast<uint8_t>(id)))
    return "unassigned";
  return rammp::axis(id).segment;
}

/// Mount NVS and read every key. Missing keys keep their defaults; that is the
/// factory state, not an error.
/// What the last load() saw from nvs_flash_init(), and whether it wiped NVS. Boot
/// logs are lost in the USB re-enumeration gap, so 'bootlog' replays these: a lost
/// encoder offset (enc cal=0) otherwise looks the same whether NVS was unreadable
/// for one boot or erased.
inline esp_err_t g_init_err = ESP_OK;
inline bool g_wiped = false;

inline bool load(AxisConfig &cfg, espp::Logger &logger) {
  esp_err_t err = nvs_flash_init();
  g_init_err = err;
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // Partition layout changed under us: wipe and start over. Provisioning is
    // lost, which the console will show as 'unassigned'.
    nvs_flash_erase();
    g_wiped = true;
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    logger.error("nvs: init failed ({})", esp_err_to_name(err));
    return false;
  }
  nvs_handle_t h;
  if (nvs_open(kNamespace, NVS_READONLY, &h) != ESP_OK) {
    return true; // namespace not created yet: factory defaults
  }
  uint8_t id = 255;
  if (nvs_get_u8(h, kKeyAxis, &id) == ESP_OK && valid_axis(id)) {
    cfg.axis_id = static_cast<rammp::AxisId>(id);
  }
  size_t len = sizeof(float);
  float ofs = NAN;
  if (nvs_get_blob(h, kKeyEncOfs, &ofs, &len) == ESP_OK && len == sizeof(float)) {
    cfg.enc_ofs_rad = ofs;
  }
  len = sizeof(cfg.hall_deg);
  std::array<float, 6> hall{};
  if (nvs_get_blob(h, kKeyHall, hall.data(), &len) == ESP_OK && len == sizeof(hall)) {
    cfg.hall_deg = hall;
  }
  uint8_t fb = 0;
  if (nvs_get_u8(h, kKeyFeedback, &fb) == ESP_OK && fb <= (uint8_t)Feedback::ENCODER) {
    cfg.feedback = static_cast<Feedback>(fb);
  }
  auto get_f = [&](const char *key, float &out) {
    size_t n = sizeof(float);
    float v = NAN;
    if (nvs_get_blob(h, key, &v, &n) == ESP_OK && n == sizeof(float))
      out = v;
  };
  get_f(kKeySpdKp, cfg.spd_kp);
  get_f(kKeySpdKi, cfg.spd_ki);
  get_f(kKeyPllHz, cfg.pll_hz);
  nvs_close(h);
  return true;
}

namespace detail {
template <typename F> inline bool write(espp::Logger &logger, F &&fn) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    logger.error("nvs: open failed ({})", esp_err_to_name(err));
    return false;
  }
  err = fn(h);
  if (err == ESP_OK)
    err = nvs_commit(h);
  nvs_close(h);
  if (err != ESP_OK) {
    logger.error("nvs: write failed ({})", esp_err_to_name(err));
    return false;
  }
  return true;
}
} // namespace detail

inline bool save_axis(rammp::AxisId id, espp::Logger &logger) {
  return detail::write(logger,
                       [&](nvs_handle_t h) { return nvs_set_u8(h, kKeyAxis, (uint8_t)id); });
}

inline bool save_enc_ofs(float ofs_rad, espp::Logger &logger) {
  return detail::write(
      logger, [&](nvs_handle_t h) { return nvs_set_blob(h, kKeyEncOfs, &ofs_rad, sizeof(ofs_rad)); });
}

inline bool save_hall(const std::array<float, 6> &deg, espp::Logger &logger) {
  return detail::write(
      logger, [&](nvs_handle_t h) { return nvs_set_blob(h, kKeyHall, deg.data(), sizeof(deg)); });
}

inline bool save_feedback(Feedback fb, espp::Logger &logger) {
  return detail::write(logger,
                       [&](nvs_handle_t h) { return nvs_set_u8(h, kKeyFeedback, (uint8_t)fb); });
}

/// Speed-loop gains and hall PLL bandwidth, in the loop's own units.
inline bool save_tuning(float kp_rpm, float ki_rpm, float pll_hz, espp::Logger &logger) {
  return detail::write(logger, [&](nvs_handle_t h) {
    esp_err_t e = nvs_set_blob(h, kKeySpdKp, &kp_rpm, sizeof(kp_rpm));
    if (e == ESP_OK)
      e = nvs_set_blob(h, kKeySpdKi, &ki_rpm, sizeof(ki_rpm));
    if (e == ESP_OK)
      e = nvs_set_blob(h, kKeyPllHz, &pll_hz, sizeof(pll_hz));
    return e;
  });
}

/// Back to the compiled defaults at the next boot. 'tune clear' on the console.
inline bool clear_tuning(espp::Logger &logger) {
  return detail::write(logger, [&](nvs_handle_t h) {
    for (const char *k : {kKeySpdKp, kKeySpdKi, kKeyPllHz}) {
      const esp_err_t e = nvs_erase_key(h, k);
      if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND)
        return e;
    }
    return ESP_OK;
  });
}

/// Forget everything: back to factory. 'axis clear' on the console.
inline bool clear(espp::Logger &logger) {
  return detail::write(logger, [&](nvs_handle_t h) { return nvs_erase_all(h); });
}

} // namespace axiscfg
