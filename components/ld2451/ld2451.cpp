#include "ld2451.h"
#include <cstring>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

// -----------------------------------------------------------------------
// HLK-LD2451 protocol notes (verified against the official Hi-Link
// "HLK-LD2451 Serial communication protocol" datasheet and cross-checked
// against a working third-party implementation):
//
// Command frame (host -> radar):
//   FD FC FB FA | len(2, LE) | cmd(1) 00 [value bytes...] | 04 03 02 01
// ACK frame (radar -> host):
//   FD FC FB FA | len(2, LE) | cmd(1) 01 [status(2, LE)] [extra...] | 04 03 02 01
//   status == 0x0000 -> success, anything else -> failure
//
// Report frame (radar -> host, streamed continuously):
//   F4 F3 F2 F1 | len(2, LE) | [target_count(1) alarm(1) target[count]*5] | F8 F7 F6 F5
//   When no target is present, len == 0 (no payload at all).
//   Per-target payload (5 bytes): angle(1, raw-0x80) distance(1, m)
//                                  direction(1, 0=away 1=toward) speed(1, km/h) snr(1)
// -----------------------------------------------------------------------

namespace esphome {
namespace ld2451 {

static const char *const TAG = "ld2451";
static const char *const COMPONENT_VERSION = "1.0.1";

static const uint8_t CMD_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};
static const uint8_t REPORT_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t REPORT_FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};

static const uint8_t CMD_ENABLE_CONFIG = 0xFF;
static const uint8_t CMD_END_CONFIG = 0xFE;
static const uint8_t CMD_READ_FIRMWARE = 0xA0;
static const uint8_t CMD_SET_BAUD_RATE = 0xA1;
static const uint8_t CMD_FACTORY_RESET = 0xA2;
static const uint8_t CMD_RESTART = 0xA3;
// Not directly confirmed in an LD2451-specific protocol excerpt - inferred
// from the LD2410's command set, which shares this exact 0xA0-0xA3 numbering
// scheme (firmware/baud/reset/restart) and uses 0xA4 for its Bluetooth
// on/off toggle with the same 2-byte {enable, 0x00} payload. Worth
// confirming against real hardware behavior.
static const uint8_t CMD_BLUETOOTH = 0xA4;
static const uint8_t CMD_SET_TARGET_DETECTION_CFG = 0x02;
static const uint8_t CMD_GET_TARGET_DETECTION_CFG = 0x12;
static const uint8_t CMD_SET_SENSITIVITY = 0x03;
static const uint8_t CMD_GET_SENSITIVITY = 0x13;

void LD2451Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LD2451...");
  this->drain_rx_();
  this->refresh_config();
  // Publish an initial "no target" state so sensors don't sit at NaN/unknown
  // until the first report frame arrives (or the idle timeout fires).
  this->clear_all_targets_();
}

void LD2451Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2451:");
  ESP_LOGCONFIG(TAG, "  Version: %s", COMPONENT_VERSION);
  if (!this->firmware_version_.empty()) {
    ESP_LOGCONFIG(TAG, "  Firmware: %s", this->firmware_version_.c_str());
  }
  ESP_LOGCONFIG(TAG, "  Max distance: %u m", this->cfg_max_distance_);
  ESP_LOGCONFIG(TAG, "  Min speed: %u km/h", this->cfg_min_speed_);
  ESP_LOGCONFIG(TAG, "  No-target delay: %u s", this->cfg_no_target_delay_);
  ESP_LOGCONFIG(TAG, "  Detection direction: %u", (unsigned) this->cfg_direction_);
  ESP_LOGCONFIG(TAG, "  SNR threshold: %u", this->cfg_snr_threshold_);
  ESP_LOGCONFIG(TAG, "  Multi-trigger required: %s", YESNO(this->cfg_multi_trigger_));
  ESP_LOGCONFIG(TAG, "  Bluetooth: %s (assumed state - not read back from radar)", ONOFF(this->cfg_bluetooth_enabled_));
}

void LD2451Component::loop() {
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b))
      break;
    this->process_byte_(b);
  }

  // Some HLK-LD2451 units simply stop transmitting on UART while no target
  // is present, rather than continuously sending the zero-length "all
  // clear" report frame the datasheet implies. Without this, has_target
  // (and everything derived from it) would latch true forever once a
  // target is seen, since nothing would ever arrive to tell us it's gone.
  // If we haven't seen *any* report frame in a while, treat that silence
  // itself as "no target" and clear state once.
  if (this->last_report_ms_ != 0 && !this->idle_cleared_ &&
      millis() - this->last_report_ms_ > IDLE_TIMEOUT_MS) {
    this->clear_all_targets_();
    this->idle_cleared_ = true;
  }
}

void LD2451Component::drain_rx_() {
  uint8_t b;
  while (this->available()) {
    this->read_byte(&b);
  }
}

// ---------------------------------------------------------------------
// Streaming report-frame parser
// ---------------------------------------------------------------------

void LD2451Component::process_byte_(uint8_t b) {
  switch (this->state_) {
    case ParseState::HEADER_1:
      this->state_ = (b == REPORT_HEADER[0]) ? ParseState::HEADER_2 : ParseState::HEADER_1;
      break;
    case ParseState::HEADER_2:
      this->state_ = (b == REPORT_HEADER[1]) ? ParseState::HEADER_3 : ParseState::HEADER_1;
      break;
    case ParseState::HEADER_3:
      this->state_ = (b == REPORT_HEADER[2]) ? ParseState::HEADER_4 : ParseState::HEADER_1;
      break;
    case ParseState::HEADER_4:
      if (b == REPORT_HEADER[3]) {
        this->state_ = ParseState::LEN_LOW;
      } else {
        this->state_ = ParseState::HEADER_1;
      }
      break;
    case ParseState::LEN_LOW:
      this->payload_len_ = b;
      this->state_ = ParseState::LEN_HIGH;
      break;
    case ParseState::LEN_HIGH:
      this->payload_len_ |= (static_cast<uint16_t>(b) << 8);
      this->payload_.clear();
      if (this->payload_len_ == 0) {
        // No target present - frame has no payload, go straight to footer.
        this->state_ = ParseState::FOOTER_1;
      } else if (this->payload_len_ > 2 + LD2451_MAX_TARGETS * 5) {
        // Sanity check - malformed/garbage length, resync.
        ESP_LOGW(TAG, "Report frame length %u out of range, resyncing", this->payload_len_);
        this->state_ = ParseState::HEADER_1;
      } else {
        this->payload_.reserve(this->payload_len_);
        this->state_ = ParseState::PAYLOAD;
      }
      break;
    case ParseState::PAYLOAD:
      this->payload_.push_back(b);
      if (this->payload_.size() >= this->payload_len_) {
        this->state_ = ParseState::FOOTER_1;
      }
      break;
    case ParseState::FOOTER_1:
      this->state_ = (b == REPORT_FOOTER[0]) ? ParseState::FOOTER_2 : ParseState::HEADER_1;
      break;
    case ParseState::FOOTER_2:
      this->state_ = (b == REPORT_FOOTER[1]) ? ParseState::FOOTER_3 : ParseState::HEADER_1;
      break;
    case ParseState::FOOTER_3:
      this->state_ = (b == REPORT_FOOTER[2]) ? ParseState::FOOTER_4 : ParseState::HEADER_1;
      break;
    case ParseState::FOOTER_4:
      if (b == REPORT_FOOTER[3]) {
        this->handle_report_payload_(this->payload_.data(), this->payload_.size());
      } else {
        ESP_LOGW(TAG, "Report frame footer mismatch, dropping frame");
      }
      this->state_ = ParseState::HEADER_1;
      break;
  }
}

void LD2451Component::handle_report_payload_(const uint8_t *data, uint16_t len) {
  this->last_report_ms_ = millis();
  this->idle_cleared_ = false;

  uint8_t count = 0;
  if (len >= 2) {
    count = data[0];
    // data[1] is the "approaching target present" alarm flag - derivable
    // from the parsed targets below, so we don't need to store it separately.
    if (count > LD2451_MAX_TARGETS)
      count = LD2451_MAX_TARGETS;
    if (2 + count * 5u > len) {
      ESP_LOGW(TAG, "Report payload too short for %u targets", count);
      count = 0;
    }
  }

  this->publish_targets_(data, count);
}

void LD2451Component::clear_all_targets_() {
  // Same code path as an actual zero-target report frame, just triggered by
  // UART silence (see loop()) instead of a received frame. Deliberately
  // does NOT touch last_report_ms_/idle_cleared_ - those are owned by
  // handle_report_payload_() and loop() respectively.
  this->publish_targets_(nullptr, 0);
}

void LD2451Component::publish_targets_(const uint8_t *data, uint8_t count) {
  bool any_target = false;
  bool any_approaching = false;

  for (uint8_t i = 0; i < LD2451_MAX_TARGETS; i++) {
    LD2451Target &t = this->targets_[i];
    if (i < count) {
      const uint8_t *base = data + 2 + i * 5;
      t.valid = true;
      t.angle = static_cast<int8_t>(static_cast<int16_t>(base[0]) - 0x80);
      t.distance = base[1];
      t.direction = base[2] == 0x01 ? LD2451Direction::TOWARD : LD2451Direction::AWAY;
      t.speed = base[3];
      t.snr = base[4];
      any_target = true;
      if (t.direction == LD2451Direction::TOWARD)
        any_approaching = true;
    } else {
      t.valid = false;
    }

#ifdef USE_SENSOR
    if (this->target_angle_sensors_[i] != nullptr) {
      if (t.valid) {
        this->target_angle_sensors_[i]->publish_state(t.angle);
      } else if (!std::isnan(this->target_angle_sensors_[i]->get_raw_state())) {
        this->target_angle_sensors_[i]->publish_state(NAN);
      }
    }
    if (this->target_distance_sensors_[i] != nullptr) {
      if (t.valid) {
        this->target_distance_sensors_[i]->publish_state(t.distance);
      } else if (!std::isnan(this->target_distance_sensors_[i]->get_raw_state())) {
        this->target_distance_sensors_[i]->publish_state(NAN);
      }
    }
    if (this->target_speed_sensors_[i] != nullptr) {
      if (t.valid) {
        // Report speed as signed: positive = approaching, negative = away.
        float signed_speed = t.direction == LD2451Direction::TOWARD ? (float) t.speed : -((float) t.speed);
        this->target_speed_sensors_[i]->publish_state(signed_speed);
      } else if (!std::isnan(this->target_speed_sensors_[i]->get_raw_state())) {
        this->target_speed_sensors_[i]->publish_state(NAN);
      }
    }
#endif
#ifdef USE_BINARY_SENSOR
    if (this->target_approaching_binary_sensors_[i] != nullptr) {
      this->target_approaching_binary_sensors_[i]->publish_state(t.valid && t.direction == LD2451Direction::TOWARD);
    }
#endif
  }

#ifdef USE_SENSOR
  if (this->target_count_sensor_ != nullptr) {
    this->target_count_sensor_->publish_state(count);
  }
#endif
#ifdef USE_BINARY_SENSOR
  if (this->has_target_binary_sensor_ != nullptr) {
    this->has_target_binary_sensor_->publish_state(any_target);
  }
  if (this->has_approaching_target_binary_sensor_ != nullptr) {
    this->has_approaching_target_binary_sensor_->publish_state(any_approaching);
  }
#endif
}

// ---------------------------------------------------------------------
// Command / ACK exchange (blocking, only used for configuration)
// ---------------------------------------------------------------------

bool LD2451Component::write_command_frame_(uint8_t command, const uint8_t *value, uint8_t value_len) {
  uint16_t inner_len = 2 + value_len;  // command word (2 bytes: cmd, 0x00) + value
  this->write_array(CMD_HEADER, 4);
  this->write_byte(inner_len & 0xFF);
  this->write_byte((inner_len >> 8) & 0xFF);
  this->write_byte(command);
  this->write_byte(0x00);
  if (value_len > 0 && value != nullptr) {
    this->write_array(value, value_len);
  }
  this->write_array(CMD_FOOTER, 4);
  this->flush();
  return true;
}

bool LD2451Component::read_ack_frame_(uint8_t command, std::vector<uint8_t> &out) {
  uint32_t start = millis();
  uint8_t match = 0;

  // Sync to FD FC FB FA, ignoring any report frames or noise in between.
  while (match < 4) {
    if (millis() - start > COMMAND_TIMEOUT_MS) {
      ESP_LOGW(TAG, "Timed out waiting for ACK header (cmd 0x%02X)", command);
      return false;
    }
    if (!this->available())
      continue;
    uint8_t b;
    if (!this->read_byte(&b))
      continue;
    match = (b == CMD_HEADER[match]) ? match + 1 : (b == CMD_HEADER[0] ? 1 : 0);
  }

  uint8_t len_buf[2];
  for (uint8_t i = 0; i < 2; i++) {
    if (!this->read_array(len_buf + i, 1)) {
      uint32_t wait_start = millis();
      while (!this->available()) {
        if (millis() - wait_start > COMMAND_TIMEOUT_MS)
          return false;
      }
      this->read_array(len_buf + i, 1);
    }
  }
  uint16_t inner_len = len_buf[0] | (static_cast<uint16_t>(len_buf[1]) << 8);
  if (inner_len < 4 || inner_len > 64) {
    ESP_LOGW(TAG, "ACK frame length %u out of range", inner_len);
    return false;
  }

  std::vector<uint8_t> inner(inner_len);
  for (uint16_t i = 0; i < inner_len; i++) {
    uint32_t wait_start = millis();
    while (!this->available()) {
      if (millis() - wait_start > COMMAND_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Timed out reading ACK payload");
        return false;
      }
    }
    this->read_array(&inner[i], 1);
  }

  uint8_t footer[4];
  for (uint8_t i = 0; i < 4; i++) {
    uint32_t wait_start = millis();
    while (!this->available()) {
      if (millis() - wait_start > COMMAND_TIMEOUT_MS)
        return false;
    }
    this->read_array(footer + i, 1);
  }
  if (memcmp(footer, CMD_FOOTER, 4) != 0) {
    ESP_LOGW(TAG, "ACK footer mismatch");
    return false;
  }

  if (inner[0] != command || inner[1] != 0x01) {
    ESP_LOGW(TAG, "ACK cmd mismatch: got 0x%02X, expected 0x%02X", inner[0], command);
    return false;
  }
  uint16_t status = inner[2] | (static_cast<uint16_t>(inner[3]) << 8);
  if (status != 0x0000) {
    ESP_LOGW(TAG, "Command 0x%02X failed (status %u)", command, status);
    return false;
  }

  out.assign(inner.begin() + 4, inner.end());
  return true;
}

bool LD2451Component::send_command_(uint8_t command, const uint8_t *value, uint8_t value_len,
                                     std::vector<uint8_t> &response) {
  this->write_command_frame_(command, value, value_len);
  return this->read_ack_frame_(command, response);
}

bool LD2451Component::enable_config_() {
  std::vector<uint8_t> resp;
  const uint8_t val[2] = {0x01, 0x00};
  // The radar may still be mid-way through streaming a report frame when we
  // ask it to enter config mode; send it twice with a short pause, as
  // recommended by the datasheet, and clear any stray bytes first.
  this->drain_rx_();
  this->send_command_(CMD_ENABLE_CONFIG, val, 2, resp);
  delay(100);  // NOLINT
  this->drain_rx_();
  return this->send_command_(CMD_ENABLE_CONFIG, val, 2, resp);
}

bool LD2451Component::end_config_() {
  std::vector<uint8_t> resp;
  return this->send_command_(CMD_END_CONFIG, nullptr, 0, resp);
}

void LD2451Component::query_firmware_version_() {
  // Response payload (after status, which send_command_/read_ack_frame_
  // already stripped and validated): 2-byte firmware type (LE, should be
  // 0x2451) + 2-byte major version + 4-byte minor version.
  std::vector<uint8_t> resp;
  if (!this->send_command_(CMD_READ_FIRMWARE, nullptr, 0, resp) || resp.size() < 8) {
    ESP_LOGW(TAG, "query_firmware_version_: failed to read firmware version");
    return;
  }

  uint16_t fw_type = resp[0] | (static_cast<uint16_t>(resp[1]) << 8);
  if (fw_type != 0x2451) {
    ESP_LOGW(TAG, "query_firmware_version_: unexpected firmware type 0x%04X (expected 0x2451)", fw_type);
  }

  char buf[32];
  // Mirrors the "Vmajor_hi.major_lo.minor" style version string used in the
  // datasheet/app (e.g. V1.01.24051510). Byte order verified against a
  // working third-party implementation, but the exact zero-padding/format
  // has not been cross-checked against a real module - if this looks off
  // against what the HLKRadarTool app reports for your unit, the raw bytes
  // are logged below so the format is easy to correct.
  snprintf(buf, sizeof(buf), "V%u.%02u.%02u%02u%02u%02u", resp[3], resp[2], resp[7], resp[6], resp[5], resp[4]);
  this->firmware_version_ = buf;

  ESP_LOGD(TAG, "Firmware version raw bytes: %02X %02X %02X %02X %02X %02X %02X %02X", resp[0], resp[1], resp[2],
           resp[3], resp[4], resp[5], resp[6], resp[7]);

#ifdef USE_TEXT_SENSOR
  if (this->firmware_version_text_sensor_ != nullptr) {
    this->firmware_version_text_sensor_->publish_state(this->firmware_version_);
  }
#endif
}

// ---------------------------------------------------------------------
// Public configuration API
// ---------------------------------------------------------------------

void LD2451Component::refresh_config() {
  if (!this->enable_config_()) {
    ESP_LOGW(TAG, "refresh_config: failed to enter config mode");
    return;
  }

  this->query_firmware_version_();

  std::vector<uint8_t> resp;
  if (this->send_command_(CMD_GET_TARGET_DETECTION_CFG, nullptr, 0, resp) && resp.size() >= 4) {
    this->cfg_max_distance_ = resp[0];
    this->cfg_direction_ = static_cast<LD2451Direction>(resp[1]);
    this->cfg_min_speed_ = resp[2];
    this->cfg_no_target_delay_ = resp[3];
  } else {
    ESP_LOGW(TAG, "refresh_config: failed to read target detection config");
  }

  resp.clear();
  if (this->send_command_(CMD_GET_SENSITIVITY, nullptr, 0, resp) && resp.size() >= 2) {
    this->cfg_multi_trigger_ = resp[0] == 0x01;
    this->cfg_snr_threshold_ = resp[1];
  } else {
    ESP_LOGW(TAG, "refresh_config: failed to read sensitivity config");
  }

  this->end_config_();
  this->dump_config();
}

void LD2451Component::set_max_distance(uint8_t meters) {
  if (meters < 10)
    meters = 10;
  if (meters > 100)
    meters = 100;
  if (!this->enable_config_())
    return;
  const uint8_t val[4] = {meters, static_cast<uint8_t>(this->cfg_direction_), this->cfg_min_speed_,
                           this->cfg_no_target_delay_};
  std::vector<uint8_t> resp;
  if (this->send_command_(CMD_SET_TARGET_DETECTION_CFG, val, 4, resp))
    this->cfg_max_distance_ = meters;
  this->end_config_();
}

void LD2451Component::set_min_speed(uint8_t kmh) {
  if (kmh > 0x78)
    kmh = 0x78;
  if (!this->enable_config_())
    return;
  const uint8_t val[4] = {this->cfg_max_distance_, static_cast<uint8_t>(this->cfg_direction_), kmh,
                           this->cfg_no_target_delay_};
  std::vector<uint8_t> resp;
  if (this->send_command_(CMD_SET_TARGET_DETECTION_CFG, val, 4, resp))
    this->cfg_min_speed_ = kmh;
  this->end_config_();
}

void LD2451Component::set_no_target_delay(uint8_t seconds) {
  if (!this->enable_config_())
    return;
  const uint8_t val[4] = {this->cfg_max_distance_, static_cast<uint8_t>(this->cfg_direction_), this->cfg_min_speed_,
                           seconds};
  std::vector<uint8_t> resp;
  if (this->send_command_(CMD_SET_TARGET_DETECTION_CFG, val, 4, resp))
    this->cfg_no_target_delay_ = seconds;
  this->end_config_();
}

void LD2451Component::set_detection_direction(LD2451Direction direction) {
  if (!this->enable_config_())
    return;
  const uint8_t val[4] = {this->cfg_max_distance_, static_cast<uint8_t>(direction), this->cfg_min_speed_,
                           this->cfg_no_target_delay_};
  std::vector<uint8_t> resp;
  if (this->send_command_(CMD_SET_TARGET_DETECTION_CFG, val, 4, resp))
    this->cfg_direction_ = direction;
  this->end_config_();
}

void LD2451Component::set_snr_threshold(uint8_t snr) {
  if (snr < 3)
    snr = 3;
  if (snr > 8)
    snr = 8;
  if (!this->enable_config_())
    return;
  const uint8_t val[4] = {static_cast<uint8_t>(this->cfg_multi_trigger_ ? 1 : 0), snr, 0x00, 0x00};
  std::vector<uint8_t> resp;
  if (this->send_command_(CMD_SET_SENSITIVITY, val, 4, resp))
    this->cfg_snr_threshold_ = snr;
  this->end_config_();
}

void LD2451Component::set_multi_trigger(bool require_multiple) {
  if (!this->enable_config_())
    return;
  const uint8_t val[4] = {static_cast<uint8_t>(require_multiple ? 1 : 0), this->cfg_snr_threshold_, 0x00, 0x00};
  std::vector<uint8_t> resp;
  if (this->send_command_(CMD_SET_SENSITIVITY, val, 4, resp))
    this->cfg_multi_trigger_ = require_multiple;
  this->end_config_();
}

void LD2451Component::set_bluetooth_enabled(bool enable) {
  if (!this->enable_config_())
    return;
  const uint8_t val[2] = {static_cast<uint8_t>(enable ? 1 : 0), 0x00};
  std::vector<uint8_t> resp;
  bool ok = this->send_command_(CMD_BLUETOOTH, val, 2, resp);
  this->end_config_();
  if (!ok) {
    ESP_LOGW(TAG, "set_bluetooth_enabled(%s): command failed - command word 0xA4 is inferred from the LD2410's "
                  "protocol, not confirmed against an LD2451-specific datasheet excerpt, so this may not be "
                  "the right command for your unit",
             ONOFF(enable));
    return;
  }
  this->cfg_bluetooth_enabled_ = enable;
  // On the LD2410 (same command family), a Bluetooth on/off change only
  // takes effect after the radar restarts - assume the same here.
  ESP_LOGI(TAG, "Bluetooth %s - restarting module for it to take effect", ONOFF(enable));
  this->restart_module();
}

void LD2451Component::factory_reset() {
  if (!this->enable_config_())
    return;
  std::vector<uint8_t> resp;
  this->send_command_(CMD_FACTORY_RESET, nullptr, 0, resp);
  this->end_config_();
  ESP_LOGI(TAG, "Factory reset requested - restart the module for it to take effect");
}

void LD2451Component::restart_module() {
  if (!this->enable_config_())
    return;
  std::vector<uint8_t> resp;
  this->send_command_(CMD_RESTART, nullptr, 0, resp);
  // Radar restarts immediately after ACKing this; no end-config needed/possible.
}

}  // namespace ld2451
}  // namespace esphome
