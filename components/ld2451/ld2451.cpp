#include "ld2451.h"
#include <cstring>
#ifdef USE_SELECT
#include "ld2451_select.h"
#endif
#ifdef USE_NUMBER
#include "ld2451_number.h"
#endif
#include "esphome/core/application.h"
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

namespace esphome::ld2451 {

static const char *const TAG = "ld2451";
static const char *const COMPONENT_VERSION = "2.0.0";

static constexpr uint8_t CMD_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static constexpr uint8_t CMD_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};
static constexpr uint8_t REPORT_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static constexpr uint8_t REPORT_FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};

static constexpr uint8_t CMD_ENABLE_CONFIG = 0xFF;
static constexpr uint8_t CMD_END_CONFIG = 0xFE;
static constexpr uint8_t CMD_READ_FIRMWARE = 0xA0;
static constexpr uint8_t CMD_SET_BAUD_RATE = 0xA1;
static constexpr uint8_t CMD_FACTORY_RESET = 0xA2;
static constexpr uint8_t CMD_RESTART = 0xA3;
// Not directly confirmed in an LD2451-specific protocol excerpt - inferred
// from the LD2410's command set, which shares this exact 0xA0-0xA3 numbering
// scheme (firmware/baud/reset/restart) and uses 0xA4 for its Bluetooth
// on/off toggle with the same 2-byte {enable, 0x00} payload. Worth
// confirming against real hardware behavior.
static constexpr uint8_t CMD_BLUETOOTH = 0xA4;
static constexpr uint8_t CMD_SET_TARGET_DETECTION_CFG = 0x02;
static constexpr uint8_t CMD_GET_TARGET_DETECTION_CFG = 0x12;
static constexpr uint8_t CMD_SET_SENSITIVITY = 0x03;
static constexpr uint8_t CMD_GET_SENSITIVITY = 0x13;

// How long to wait, with no report frame received at all, before treating
// the radar's silence as "no target present" (see loop()). Some units
// stop transmitting entirely rather than sending an explicit zero-length
// report frame when no target is in view.
static constexpr uint32_t IDLE_TIMEOUT_MS = 1500;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 500;

// Upper bound on time spent draining the UART buffer in a single
// process_frames_() call, so a burst/backlog of report frames can't hold up
// the rest of loop() past ESPHome's non-blocking guidance. Parser state is
// resumable, so an early exit here just continues on the next loop() call.
static constexpr uint32_t FRAME_PROCESSING_BUDGET_US = 5000;

// Simple check to see if the loop start time has exceeded the allowed
// time since a saved time value.
static bool wait_time_exceeded(uint32_t last_action_ms, uint32_t timeout_ms) {
  return (App.get_loop_component_start_time() - last_action_ms) > timeout_ms;
}

// ---------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------
//
void LD2451Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LD2451...");
  this->drain_rx_();
  this->pending_commands_ = CommandFlags::READ_FIRMWARE | CommandFlags::GET_TARGET_DETECTION |
                            CommandFlags::GET_SENSITIVITY | CommandFlags::DUMP_CONFIG;
  // this->refresh_config();
  // Publish an initial "no target" state so sensors don't sit at NaN/unknown
  // until the first report frame arrives (or the idle timeout fires).
  this->clear_all_targets_();
}

// ---------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------
//
// loop() handles the sending of all control commands and the
// reading and processing of all acknowledgements or report frames
// Any control commands are marked as pending in their calls from the user
// and processed within this loop.
void LD2451Component::loop() {
  // Process pending commands
  this->action_commands_();

  // Process command responses and report frames
  this->process_frames_();

  // Some HLK-LD2451 units simply stop transmitting on UART while no target
  // is present, rather than continuously sending the zero-length "all
  // clear" report frame the datasheet implies. Without this, has_target
  // (and everything derived from it) would latch true forever once a
  // target is seen, since nothing would ever arrive to tell us it's gone.
  // If we haven't seen *any* report frame in a while, treat that silence
  // itself as "no target" and clear state once.
  if (this->last_report_ms_ != 0 && !this->idle_cleared_ &&
      !wait_time_exceeded(this->last_report_ms_, IDLE_TIMEOUT_MS)) {
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
// Commands Action Processor
// ---------------------------------------------------------------------
//

// Called once the Begin Configuration frame has been successful
// Once in Command Mode we can send any pending commands before exiting the mode
//
void LD2451Component::action_commands_() {
  // uint8_t command;
  // uint8_t val[] = {0x00, 0x00, 0x00, 0x00};
  switch (this->command_state_) {
    // Wait for a response from command action
    // Timeout if no response recieved within defined period
    case CommandState::WAIT_RESPONSE:
      if (wait_time_exceeded(this->last_action_ms_, COMMAND_TIMEOUT_MS)) {
        // pending_commands_ is only empty here while waiting on the END_CONFIG
        // ack - retry that specifically instead of dropping back to idle.
        this->command_state_ = this->pending_commands_ ? CommandState::BEGIN_CONFIG : CommandState::END_CONFIG;
        ESP_LOGV(TAG, "Command (0x%04X) response timed out: restarting", this->pending_commands_);
      }
      break;

    case CommandState::BEGIN_CONFIG:
      if (!this->pending_commands_)
        break;  // exit if all commands complete
      this->begin_config_();
      this->command_state_ = CommandState::WAIT_RESPONSE;
      this->last_action_ms_ = App.get_loop_component_start_time();
      break;

    case CommandState::SEND_COMMAND:
      // The order of the if statement is important e.g.
      // The Bluetooth change needs to happen before the module restart
      if (this->pending_commands_ & CommandFlags::FACTORY_RESET) {
        this->factory_reset_();
      } else if (this->pending_commands_ & CommandFlags::BLUETOOTH) {
        this->enable_bluetooth_();
      } else if (this->pending_commands_ & CommandFlags::RESTART) {
        this->restart_module_();
      } else if (this->pending_commands_ & CommandFlags::READ_FIRMWARE) {
        this->read_firmware_();
      } else if (this->pending_commands_ & CommandFlags::SET_SENSITIVITY) {
        this->set_sensitivity_();
      } else if (this->pending_commands_ & CommandFlags::SET_TARGET_DETECTION) {
        this->set_target_detection_cfg_();
      } else if (this->pending_commands_ & CommandFlags::GET_SENSITIVITY) {
        this->get_sensitivity_();
      } else if (this->pending_commands_ & CommandFlags::GET_TARGET_DETECTION) {
        this->get_target_detection_cfg_();
      } else if (this->pending_commands_ & CommandFlags::DUMP_CONFIG) {
        if (wait_time_exceeded(this->last_action_ms_, 100)) {
          this->dump_config();
          this->pending_commands_ &= ~CommandFlags::DUMP_CONFIG;
          this->command_state_ = this->pending_commands_ ? CommandState::SEND_COMMAND : CommandState::END_CONFIG;
        }
        break;
      }
      this->command_state_ = CommandState::WAIT_RESPONSE;
      this->last_action_ms_ = App.get_loop_component_start_time();
      break;

    case CommandState::END_CONFIG:
      this->end_config_();
      this->command_state_ = CommandState::WAIT_RESPONSE;
      this->last_action_ms_ = App.get_loop_component_start_time();
      break;
  }
}

// ---------------------------------------------------------------------
// Command Response Frame Processing
// ---------------------------------------------------------------------
//

void LD2451Component::complete_command_(CommandFlags flag) {
  this->pending_commands_ &= ~flag;
  this->command_state_ = this->pending_commands_ ? CommandState::SEND_COMMAND : CommandState::END_CONFIG;
}

bool LD2451Component::handle_command_response_frame_(const uint8_t *data, uint16_t len) {
  const uint8_t command = data[0];
  const uint16_t result = (data[2] | data[3] << 8);

  // Check for command failure - result non-zero
  // Commands keep retrying until they are successful
  if (result) {
    ESP_LOGW(TAG, "Command %02X failed. Result: %04X", command, result);
    return false;
  }

  // Process succesful command
  switch (command) {
    case CMD_ENABLE_CONFIG:
      if (this->comms_protocol_version_ == 0xFFFF) {
        this->comms_protocol_version_ = (data[4] | data[5] << 8);
      }
      this->command_state_ = CommandState::SEND_COMMAND;
      break;

    case CMD_END_CONFIG:
      this->command_state_ = CommandState::BEGIN_CONFIG;
      break;

    case CMD_FACTORY_RESET:
      this->pending_commands_ &= ~CommandFlags::FACTORY_RESET;
      // No need for an END_CONFIG as I am reseting the module.
      this->command_state_ = CommandState::BEGIN_CONFIG;
      break;

    case CMD_RESTART:
      this->pending_commands_ &= ~CommandFlags::RESTART;
      // No need for an END_CONFIG as I am reseting the module.
      this->command_state_ = CommandState::BEGIN_CONFIG;
      break;

    case CMD_BLUETOOTH:
      this->complete_command_(CommandFlags::BLUETOOTH);
      break;

    case CMD_READ_FIRMWARE:
      this->complete_command_(CommandFlags::READ_FIRMWARE);
      if (data[5] != 0x24 && data[4] != 0x51) {  // firmware type 0x2451
        ESP_LOGW(TAG, "query_firmware_version_: unexpected firmware type 0x%02X%02X (expected 0x2451)", data[5],
                 data[4]);
      }

      char buf[32];
      // Mirrors the "Vmajor_hi.major_lo.minor" style version string used in the
      // datasheet/app (e.g. V1.01.24051510). Byte order verified against a
      // working third-party implementation, but the exact zero-padding/format
      // has not been cross-checked against a real module - if this looks off
      // against what the HLKRadarTool app reports for your unit, the raw bytes
      // are logged below so the format is easy to correct.
      snprintf(buf, sizeof(buf), "V%u.%02u.%02u%02u%02u%02u", data[7], data[6], data[11], data[10], data[9], data[8]);
      this->firmware_version_ = buf;

      ESP_LOGV(TAG, "Firmware version raw bytes: %02X %02X %02X %02X %02X %02X %02X %02X", data[4], data[5], data[7],
               data[6], data[11], data[10], data[9], data[8]);
#ifdef USE_TEXT_SENSOR
      if (this->firmware_version_text_sensor_ != nullptr) {
        this->firmware_version_text_sensor_->publish_state(this->firmware_version_);
      }
#endif
      break;

    case CMD_GET_SENSITIVITY:
      this->complete_command_(CommandFlags::GET_SENSITIVITY);
      // Process the data
      this->cfg_multi_trigger_ = data[4] == 0x01;
      this->cfg_snr_threshold_ = data[5];
      // TODO: Need to publish multi_trigger switch state here too
#ifdef USE_NUMBER
      if (this->snr_threshold_number_ != nullptr) {
        this->snr_threshold_number_->publish_state(this->cfg_snr_threshold_);
      }
#endif
      break;

    case CMD_SET_SENSITIVITY:
      this->complete_command_(CommandFlags::SET_SENSITIVITY);
      break;

    case CMD_GET_TARGET_DETECTION_CFG:
      this->complete_command_(CommandFlags::GET_TARGET_DETECTION);
      // Process the data
      this->cfg_max_distance_ = data[4];
      this->cfg_direction_ = static_cast<LD2451Direction>(data[5]);
      this->cfg_min_speed_ = data[6];
      this->cfg_no_target_delay_ = data[7];
#ifdef USE_NUMBER
      if (this->max_distance_number_ != nullptr) {
        this->max_distance_number_->publish_state(this->cfg_max_distance_);
      }
      if (this->min_speed_number_ != nullptr) {
        this->min_speed_number_->publish_state(this->cfg_min_speed_);
      }
      if (this->no_target_delay_number_ != nullptr) {
        this->no_target_delay_number_->publish_state(this->cfg_no_target_delay_);
      }
#endif
#ifdef USE_SELECT
      if (this->direction_select_ != nullptr) {
        this->direction_select_->publish_direction(this->cfg_direction_);
      }
#endif
      break;

    case CMD_SET_TARGET_DETECTION_CFG:
      this->complete_command_(CommandFlags::SET_TARGET_DETECTION);
      break;
  }
  return true;
}

// ---------------------------------------------------------------------
// Streaming Frame parser
// ---------------------------------------------------------------------

// Read rx data, build a frame and then process it
void LD2451Component::process_frames_() {
  const uint32_t start_us = micros();
  while (this->available()) {
    if (micros() - start_us > FRAME_PROCESSING_BUDGET_US) {
      ESP_LOGV(TAG, "process_frames_: budget exceeded, resuming next loop()");
      break;
    }

    uint8_t uart_byte;

    if (!this->read_byte(&uart_byte))
      break;

    switch (this->state_) {
      uint8_t frame_byte;
      // Decide whether this is a report frame or a command ACK frame, and switch to the appropriate state machine.
      case ParseState::HEADER_1:
        if (uart_byte == REPORT_HEADER[0]) {
          this->state_ = ParseState::HEADER_2;
          this->frame_type_ = FrameType::REPORT;
        } else if (uart_byte == CMD_HEADER[0]) {
          this->state_ = ParseState::HEADER_2;
          this->frame_type_ = FrameType::COMMAND;
        }
        break;

      case ParseState::HEADER_2:
        frame_byte = (this->frame_type_ == FrameType::COMMAND) ? CMD_HEADER[1] : REPORT_HEADER[1];
        this->state_ = (uart_byte == frame_byte) ? ParseState::HEADER_3 : ParseState::HEADER_1;
        break;

      case ParseState::HEADER_3:
        frame_byte = (this->frame_type_ == FrameType::COMMAND) ? CMD_HEADER[2] : REPORT_HEADER[2];
        this->state_ = (uart_byte == frame_byte) ? ParseState::HEADER_4 : ParseState::HEADER_1;
        break;

      case ParseState::HEADER_4:
        frame_byte = (this->frame_type_ == FrameType::COMMAND) ? CMD_HEADER[3] : REPORT_HEADER[3];
        this->state_ = (uart_byte == frame_byte) ? ParseState::LEN_LOW : ParseState::HEADER_1;
        break;

      case ParseState::LEN_LOW:
        this->payload_len_ = uart_byte;
        this->state_ = ParseState::LEN_HIGH;
        break;

      case ParseState::LEN_HIGH:
        this->payload_len_ |= (static_cast<uint16_t>(uart_byte) << 8);
        this->payload_.clear();
        if (this->payload_len_ == 0) {
          // No target present - frame has no payload, go straight to footer.
          this->state_ = ParseState::FOOTER_1;
        } else if (this->payload_len_ > 2 + LD2451_MAX_TARGETS * 5) {
          // Sanity check - malformed/garbage length, resync.
          ESP_LOGW(TAG, "Frame length %u out of range, resyncing", this->payload_len_);
          this->state_ = ParseState::HEADER_1;
        } else {
          this->payload_.reserve(this->payload_len_);
          this->state_ = ParseState::PAYLOAD;
        }
        break;

      case ParseState::PAYLOAD:
        this->payload_.push_back(uart_byte);
        if (this->payload_.size() >= this->payload_len_) {
          this->state_ = ParseState::FOOTER_1;
        }
        break;

      case ParseState::FOOTER_1:
        frame_byte = (this->frame_type_ == FrameType::COMMAND) ? CMD_FOOTER[0] : REPORT_FOOTER[0];
        this->state_ = (uart_byte == frame_byte) ? ParseState::FOOTER_2 : ParseState::HEADER_1;
        break;

      case ParseState::FOOTER_2:
        frame_byte = (this->frame_type_ == FrameType::COMMAND) ? CMD_FOOTER[1] : REPORT_FOOTER[1];
        this->state_ = (uart_byte == frame_byte) ? ParseState::FOOTER_3 : ParseState::HEADER_1;
        break;

      case ParseState::FOOTER_3:
        frame_byte = (this->frame_type_ == FrameType::COMMAND) ? CMD_FOOTER[2] : REPORT_FOOTER[2];
        this->state_ = (uart_byte == frame_byte) ? ParseState::FOOTER_4 : ParseState::HEADER_1;
        break;

      case ParseState::FOOTER_4:
        frame_byte = (this->frame_type_ == FrameType::COMMAND) ? CMD_FOOTER[3] : REPORT_FOOTER[3];
        this->state_ = ParseState::HEADER_1;
        if (uart_byte != frame_byte) {
          ESP_LOGW(TAG, "Frame footer mismatch, dropping frame");
          break;
        }
        if (this->frame_type_ == FrameType::COMMAND) {
          char hex_buf[this->payload_.size() * 3];
          ESP_LOGD(TAG, "Command response: %s (%u)",
                   format_hex_pretty_to(hex_buf, sizeof(hex_buf), this->payload_.data(), this->payload_.size()),
                   this->payload_.size());
          this->handle_command_response_frame_(this->payload_.data(), this->payload_.size());
        } else {
          char hex_buf[this->payload_.size() * 3];
          ESP_LOGV(TAG, "Report frame: %s (%u)",
                   format_hex_pretty_to(hex_buf, sizeof(hex_buf), this->payload_.data(), this->payload_.size()),
                   this->payload_.size());
          this->handle_report_frame_(this->payload_.data(), this->payload_.size());
        }
        break;
    }
  }
}

// ---------------------------------------------------------------------
// Report Frame Processing
// ---------------------------------------------------------------------
//
void LD2451Component::handle_report_frame_(const uint8_t *data, uint16_t len) {
  this->last_report_ms_ = App.get_loop_component_start_time();
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

// ---------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------
//

void LD2451Component::write_command_frame_(uint8_t command, const uint8_t *value, uint8_t value_len) {
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

  char hex[value_len * 3 + 1];
  ESP_LOGV(TAG, "Sent command: %02X, Data: %s (%d)", command, format_hex_pretty_to(hex, sizeof(hex), value, value_len),
           value_len);
}

void LD2451Component::begin_config_() {
  const uint8_t val[2] = {0x01, 0x00};
  this->write_command_frame_(CMD_ENABLE_CONFIG, val, sizeof(val));
}

void LD2451Component::end_config_() { this->write_command_frame_(CMD_END_CONFIG, nullptr, 0); }

void LD2451Component::factory_reset_() { this->write_command_frame_(CMD_FACTORY_RESET, nullptr, 0); }

void LD2451Component::restart_module_() { this->write_command_frame_(CMD_RESTART, nullptr, 0); }

void LD2451Component::read_firmware_() { this->write_command_frame_(CMD_READ_FIRMWARE, nullptr, 0); }

void LD2451Component::get_sensitivity_() { this->write_command_frame_(CMD_GET_SENSITIVITY, nullptr, 0); }

// TODO: cfg_multi_trigger should be a number between 0 and 10. Not a boolean!
void LD2451Component::set_sensitivity_() {
  const uint8_t val[4] = {static_cast<uint8_t>(this->cfg_multi_trigger_ ? 1 : 0), this->cfg_snr_threshold_, 0x00, 0x00};
  this->write_command_frame_(CMD_SET_SENSITIVITY, val, sizeof(val));
}

void LD2451Component::get_target_detection_cfg_() {
  this->write_command_frame_(CMD_GET_TARGET_DETECTION_CFG, nullptr, 0);
}

void LD2451Component::set_target_detection_cfg_() {
  const uint8_t val[4] = {this->cfg_max_distance_, static_cast<uint8_t>(this->cfg_direction_), this->cfg_min_speed_,
                          this->cfg_no_target_delay_};
  this->write_command_frame_(CMD_SET_TARGET_DETECTION_CFG, val, sizeof(val));
}

void LD2451Component::enable_bluetooth_() {
  const uint8_t val[2] = {static_cast<uint8_t>(this->cfg_bluetooth_enabled_ ? 1 : 0), 0x00};
  this->write_command_frame_(CMD_BLUETOOTH, val, sizeof(val));
}

// ---------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------
//

static const char *direction_to_string(LD2451Direction direction) {
  switch (direction) {
    case LD2451Direction::AWAY:
      return "Away";
    case LD2451Direction::TOWARD:
      return "Towards";
    case LD2451Direction::ALL:
      return "All";
  }
  return "Unknown";
}

void LD2451Component::dump_config() {
  ESP_LOGCONFIG(TAG,
                "LD2451:\n"
                "  Version: %s\n"
                "  Firmware: %s\n"
                "  Comms Protocol Version : %04X\n"
                "  Max distance: %u m\n"
                "  Min speed: %u km/h\n"
                "  No-target delay: %u s\n"
                "  Detection direction: %s\n"
                "  SNR threshold: %u\n"
                "  Multi-trigger required: %s\n"
                "  Bluetooth: %s (assumed state - not read back from radar)",
                COMPONENT_VERSION, this->firmware_version_.empty() ? "Unknown" : this->firmware_version_.c_str(),
                this->comms_protocol_version_, this->cfg_max_distance_, this->cfg_min_speed_,
                this->cfg_no_target_delay_, direction_to_string(this->cfg_direction_), this->cfg_snr_threshold_,
                YESNO(this->cfg_multi_trigger_), ONOFF(this->cfg_bluetooth_enabled_));
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
// Public configuration API
//
// NOTE: The SET commands are follwed by a GET command in order to ensure
//       the entities are updated with the value from the LD2451
// ---------------------------------------------------------------------

void LD2451Component::refresh_config() {
  this->pending_commands_ |= CommandFlags::READ_FIRMWARE | CommandFlags::GET_TARGET_DETECTION |
                             CommandFlags::GET_SENSITIVITY | CommandFlags::DUMP_CONFIG;
}

void LD2451Component::set_max_distance(uint8_t meters) {
  if (meters < 10)
    meters = 10;
  if (meters > 100)
    meters = 100;
  this->cfg_max_distance_ = meters;
  this->pending_commands_ |= (CommandFlags::SET_TARGET_DETECTION | CommandFlags::GET_TARGET_DETECTION);
}

void LD2451Component::set_min_speed(uint8_t kmh) {
  if (kmh > 0x78)
    kmh = 0x78;
  this->cfg_min_speed_ = kmh;
  this->pending_commands_ |= (CommandFlags::SET_TARGET_DETECTION | CommandFlags::GET_TARGET_DETECTION);
}

void LD2451Component::set_no_target_delay(uint8_t seconds) {
  this->cfg_no_target_delay_ = seconds;
  this->pending_commands_ |= (CommandFlags::SET_TARGET_DETECTION | CommandFlags::GET_TARGET_DETECTION);
}

void LD2451Component::set_detection_direction(LD2451Direction direction) {
  this->cfg_direction_ = direction;
  this->pending_commands_ |= (CommandFlags::SET_TARGET_DETECTION | CommandFlags::GET_TARGET_DETECTION);
}

void LD2451Component::set_snr_threshold(uint8_t snr) {
  if (snr < 3)
    snr = 3;
  if (snr > 8)
    snr = 8;
  this->cfg_snr_threshold_ = snr;
  this->pending_commands_ |= (CommandFlags::SET_SENSITIVITY | CommandFlags::GET_SENSITIVITY);
}

void LD2451Component::set_multi_trigger(bool require_multiple) {
  this->cfg_multi_trigger_ = require_multiple;
  this->pending_commands_ |= (CommandFlags::SET_SENSITIVITY | CommandFlags::GET_SENSITIVITY);
}

void LD2451Component::set_bluetooth_enable(bool enable) {
  ESP_LOGI(TAG, "Bluetooth %s - restarting module for it to take effect", ONOFF(enable));
  this->cfg_bluetooth_enabled_ = enable;
  this->pending_commands_ |= CommandFlags::BLUETOOTH;
  this->pending_commands_ |= CommandFlags::RESTART;
}

void LD2451Component::factory_reset() {
  this->pending_commands_ |= CommandFlags::FACTORY_RESET;
  ESP_LOGI(TAG, "Factory reset requested - restart the module for it to take effect");
}

void LD2451Component::restart_module() { this->pending_commands_ |= CommandFlags::RESTART; }

}  // namespace esphome::ld2451
