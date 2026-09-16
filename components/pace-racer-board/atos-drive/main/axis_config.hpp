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
//
// One NVS namespace, one key per field, all plain reads and writes; nothing
// here is timing critical. Saves happen from the console task only.

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

struct AxisConfig {
  rammp::AxisId axis_id{rammp::AxisId::UNASSIGNED};
  float enc_ofs_rad{NAN};
  std::array<float, 6> hall_deg{NAN, NAN, NAN, NAN, NAN, NAN};

  bool assigned() const { return axis_id != rammp::AxisId::UNASSIGNED; }
  bool enc_calibrated() const { return !std::isnan(enc_ofs_rad); }
  bool hall_calibrated() const {
    for (float d : hall_deg)
      if (std::isnan(d))
        return false;
    return true;
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
inline bool load(AxisConfig &cfg, espp::Logger &logger) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // Partition layout changed under us: wipe and start over. Provisioning is
    // lost, which the console will show as 'unassigned'.
    nvs_flash_erase();
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

/// Forget everything: back to factory. 'axis clear' on the console.
inline bool clear(espp::Logger &logger) {
  return detail::write(logger, [&](nvs_handle_t h) { return nvs_erase_all(h); });
}

} // namespace axiscfg
