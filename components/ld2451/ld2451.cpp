#include "ld2451.h"
#include <cmath>
#include <cstdio>
#ifdef USE_SELECT
#include "ld2451_select.h"
#endif
#ifdef USE_NUMBER
#include "ld2451_number.h"
#endif
#ifdef USE_SWITCH
#include "ld2451_switch.h"
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
static const char *const COMPONENT_VERSION = "2.1.1dev";

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
// Attempts per command (timeouts or failure replies) before it is dropped.
static constexpr uint8_t MAX_COMMAND_ATTEMPTS = 3;
// Time allowed for the radar to boot after a restart before sending it more
// commands. Not specified in the datasheet - chosen conservatively.
static constexpr uint32_t RESTART_SETTLE_MS = 2000;

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
#ifdef USE_TEXT_SENSOR
  // Compile-time constant, so publish it immediately rather than waiting on the radar.
  if (this->component_version_text_sensor_ != nullptr) {
    this->component_version_text_sensor_->publish_state(COMPONENT_VERSION);
  }
#endif
  this->pending_commands_ = CommandFlags::COMMAND_FLAG_READ_FIRMWARE | CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION |
                            CommandFlags::COMMAND_FLAG_GET_SENSITIVITY | CommandFlags::COMMAND_FLAG_DUMP_CONFIG;
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
      wait_time_exceeded(this->last_report_ms_, IDLE_TIMEOUT_MS)) {
    ESP_LOGD(TAG, "No report frames. Clear all targets");
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
  switch (this->command_state_) {
    // Wait for a response from command action
    // Timeout if no response received within defined period
    case CommandState::COMMAND_STATE_WAIT_RESPONSE:
      if (wait_time_exceeded(this->last_action_ms_, COMMAND_TIMEOUT_MS)) {
        ESP_LOGV(TAG, "Command %02X response timed out", this->in_flight_cmd_);
        this->command_failed_();
      }
      break;

    case CommandState::COMMAND_STATE_BEGIN_CONFIG:
      if (!this->pending_commands_)
        break;  // exit if all commands complete
      if (this->restarting_) {
        if (!wait_time_exceeded(this->restart_ms_, RESTART_SETTLE_MS))
          break;  // radar still booting
        this->restarting_ = false;
      }
      this->begin_config_();
      this->command_state_ = CommandState::COMMAND_STATE_WAIT_RESPONSE;
      this->last_action_ms_ = App.get_loop_component_start_time();
      break;

    case CommandState::COMMAND_STATE_SEND_COMMAND:
      // The order of the if statement is important e.g.
      // The Bluetooth change needs to happen before the module restart
      if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_FACTORY_RESET) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_FACTORY_RESET;
        this->factory_reset_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_BLUETOOTH) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_BLUETOOTH;
        this->enable_bluetooth_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_RESTART) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_RESTART;
        this->restart_module_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_READ_FIRMWARE) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_READ_FIRMWARE;
        this->read_firmware_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_SET_SENSITIVITY) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_SET_SENSITIVITY;
        this->set_sensitivity_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_SET_TARGET_DETECTION) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_SET_TARGET_DETECTION;
        this->set_target_detection_cfg_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_GET_SENSITIVITY) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_GET_SENSITIVITY;
        this->get_sensitivity_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION) {
        this->in_flight_flag_ = CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION;
        this->get_target_detection_cfg_();
      } else if (this->pending_commands_ & CommandFlags::COMMAND_FLAG_DUMP_CONFIG) {
        if (wait_time_exceeded(this->last_action_ms_, 100)) {
          this->dump_config();
          this->pending_commands_ &= ~CommandFlags::COMMAND_FLAG_DUMP_CONFIG;
          this->command_state_ = this->pending_commands_ ? CommandState::COMMAND_STATE_SEND_COMMAND
                                                         : CommandState::COMMAND_STATE_END_CONFIG;
        }
        break;
      }
      this->command_state_ = CommandState::COMMAND_STATE_WAIT_RESPONSE;
      this->last_action_ms_ = App.get_loop_component_start_time();
      break;

    case CommandState::COMMAND_STATE_END_CONFIG:
      this->end_config_();
      this->command_state_ = CommandState::COMMAND_STATE_WAIT_RESPONSE;
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
  this->command_state_ =
      this->pending_commands_ ? CommandState::COMMAND_STATE_SEND_COMMAND : CommandState::COMMAND_STATE_END_CONFIG;
}

// Called when the in-flight command times out or the radar reports a failure.
// Retries (re-entering config mode first) up to MAX_COMMAND_ATTEMPTS, then
// gives up on that command so a bad command can't keep the radar in config
// mode forever.
void LD2451Component::command_failed_() {
  if (++this->attempts_ < MAX_COMMAND_ATTEMPTS) {
    // pending_commands_ is only empty here while waiting on the END_CONFIG ack
    this->command_state_ =
        this->pending_commands_ ? CommandState::COMMAND_STATE_BEGIN_CONFIG : CommandState::COMMAND_STATE_END_CONFIG;
    return;
  }

  this->attempts_ = 0;
  this->status_set_warning();
  switch (this->in_flight_cmd_) {
    case CMD_ENABLE_CONFIG:
      // Radar isn't responding at all - drop everything until the next request.
      ESP_LOGW(TAG, "No response from radar, dropping pending commands (0x%04X)", this->pending_commands_);
      this->pending_commands_ = 0;
      this->command_state_ = CommandState::COMMAND_STATE_BEGIN_CONFIG;
      break;
    case CMD_END_CONFIG:
      ESP_LOGW(TAG, "End config failed, giving up");
      this->command_state_ = CommandState::COMMAND_STATE_BEGIN_CONFIG;
      break;
    default:
      ESP_LOGW(TAG, "Command %02X failed after %u attempts, skipping", this->in_flight_cmd_, MAX_COMMAND_ATTEMPTS);
      this->pending_commands_ &= ~this->in_flight_flag_;
      this->command_state_ =
          this->pending_commands_ ? CommandState::COMMAND_STATE_BEGIN_CONFIG : CommandState::COMMAND_STATE_END_CONFIG;
      break;
  }
}

// Minimum ACK payload length for each command: cmd(2) + status(2) + any
// data bytes read by handle_command_response_frame_().
static uint16_t min_ack_len(uint8_t command) {
  switch (command) {
    case CMD_ENABLE_CONFIG:
    case CMD_GET_SENSITIVITY:
      return 6;
    case CMD_GET_TARGET_DETECTION_CFG:
      return 8;
    case CMD_READ_FIRMWARE:
      return 12;
    default:
      return 4;
  }
}

bool LD2451Component::handle_command_response_frame_(const uint8_t *data, uint16_t len) {
  // ACK frames are cmd(1) 0x01 status(2) [data...]
  if (len < 4 || data[1] != 0x01) {
    ESP_LOGW(TAG, "Invalid command response (len %u), ignoring", len);
    return false;
  }
  const uint8_t command = data[0];
  if (len < min_ack_len(command)) {
    ESP_LOGW(TAG, "Command %02X response too short (%u < %u), ignoring", command, len, min_ack_len(command));
    return false;
  }
  const uint16_t result = (data[2] | data[3] << 8);

  // Ignore late/stale replies to an earlier command
  if (this->command_state_ != CommandState::COMMAND_STATE_WAIT_RESPONSE || command != this->in_flight_cmd_) {
    ESP_LOGV(TAG, "Unexpected response to command %02X, ignoring", command);
    return false;
  }

  // Check for command failure - result non-zero
  if (result) {
    ESP_LOGW(TAG, "Command %02X failed. Result: %04X", command, result);
    this->command_failed_();
    return false;
  }
  // Enable-config is only the preamble to a retry, so it must not reset the
  // attempt count of the command being retried.
  if (command != CMD_ENABLE_CONFIG) {
    this->attempts_ = 0;
    this->status_clear_warning();
  }

  // Process successful command
  switch (command) {
    case CMD_ENABLE_CONFIG:
      if (this->comms_protocol_version_ == 0xFFFF) {
        this->comms_protocol_version_ = (data[4] | data[5] << 8);
      }
      this->command_state_ = CommandState::COMMAND_STATE_SEND_COMMAND;
      break;

    case CMD_END_CONFIG:
      this->command_state_ = CommandState::COMMAND_STATE_BEGIN_CONFIG;
      break;

    case CMD_FACTORY_RESET:
      // The reset only takes effect after a restart, which factory_reset()
      // has queued, so carry on in config mode to send it.
      this->complete_command_(CommandFlags::COMMAND_FLAG_FACTORY_RESET);
      // Drop any unwritten changes and assume the default Bluetooth state
      this->req_fields_ = 0;
      this->cfg_bluetooth_enabled_ = true;
      break;

    case CMD_RESTART:
      this->pending_commands_ &= ~CommandFlags::COMMAND_FLAG_RESTART;
      // No need for an END_CONFIG as the radar leaves config mode when it
      // restarts. Re-read the config once it is back, since a restart may
      // have applied a factory reset or other change.
      this->target_detection_read_ = false;
      this->sensitivity_read_ = false;
      this->pending_commands_ |= CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION |
                                 CommandFlags::COMMAND_FLAG_GET_SENSITIVITY | CommandFlags::COMMAND_FLAG_DUMP_CONFIG;
      this->restarting_ = true;
      this->restart_ms_ = App.get_loop_component_start_time();
      this->command_state_ = CommandState::COMMAND_STATE_BEGIN_CONFIG;
      break;

    case CMD_BLUETOOTH:
      this->complete_command_(CommandFlags::COMMAND_FLAG_BLUETOOTH);
      break;

    case CMD_READ_FIRMWARE:
      this->complete_command_(CommandFlags::COMMAND_FLAG_READ_FIRMWARE);
      if (uint16_t(data[4] | data[5] << 8) != 0x2451) {
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
      this->complete_command_(CommandFlags::COMMAND_FLAG_GET_SENSITIVITY);
      // Process the data
      this->cfg_trigger_count_ = data[4];
      this->cfg_snr_threshold_ = data[5];
      this->sensitivity_read_ = true;
      // Write any change requested before the config was first read
      if (this->req_fields_ & SENSITIVITY_FIELDS) {
        this->pending_commands_ |= CommandFlags::COMMAND_FLAG_SET_SENSITIVITY;
        this->command_state_ = CommandState::COMMAND_STATE_SEND_COMMAND;
      }
#ifdef USE_NUMBER
      if (this->snr_threshold_number_ != nullptr) {
        this->snr_threshold_number_->publish_state(this->cfg_snr_threshold_);
      }
      if (this->trigger_count_number_ != nullptr) {
        this->trigger_count_number_->publish_state(this->cfg_trigger_count_);
      }
#endif
#ifdef USE_SWITCH
      if (this->multi_trigger_switch_ != nullptr) {
        this->multi_trigger_switch_->publish_state(this->cfg_trigger_count_ != 0);
      }
#endif
      break;

    case CMD_SET_SENSITIVITY:
      this->config_written_(CommandFlags::COMMAND_FLAG_SET_SENSITIVITY, CommandFlags::COMMAND_FLAG_GET_SENSITIVITY,
                            SENSITIVITY_FIELDS);
      break;

    case CMD_GET_TARGET_DETECTION_CFG:
      this->complete_command_(CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION);
      // Process the data
      this->cfg_max_distance_ = data[4];
      this->cfg_direction_ = static_cast<LD2451Direction>(data[5]);
      this->cfg_min_speed_ = data[6];
      this->cfg_no_target_delay_ = data[7];
      this->target_detection_read_ = true;
      // Write any change requested before the config was first read
      if (this->req_fields_ & TARGET_DETECTION_FIELDS) {
        this->pending_commands_ |= CommandFlags::COMMAND_FLAG_SET_TARGET_DETECTION;
        this->command_state_ = CommandState::COMMAND_STATE_SEND_COMMAND;
      }
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
      this->config_written_(CommandFlags::COMMAND_FLAG_SET_TARGET_DETECTION,
                            CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION, TARGET_DETECTION_FIELDS);
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
      case ParseState::PARSE_STATE_HEADER_1:
        if (uart_byte == REPORT_HEADER[0]) {
          this->state_ = ParseState::PARSE_STATE_HEADER_2;
          this->frame_type_ = FrameType::FRAME_TYPE_REPORT;
        } else if (uart_byte == CMD_HEADER[0]) {
          this->state_ = ParseState::PARSE_STATE_HEADER_2;
          this->frame_type_ = FrameType::FRAME_TYPE_COMMAND;
        }
        break;

      case ParseState::PARSE_STATE_HEADER_2:
        frame_byte = (this->frame_type_ == FrameType::FRAME_TYPE_COMMAND) ? CMD_HEADER[1] : REPORT_HEADER[1];
        this->state_ = (uart_byte == frame_byte) ? ParseState::PARSE_STATE_HEADER_3 : ParseState::PARSE_STATE_HEADER_1;
        break;

      case ParseState::PARSE_STATE_HEADER_3:
        frame_byte = (this->frame_type_ == FrameType::FRAME_TYPE_COMMAND) ? CMD_HEADER[2] : REPORT_HEADER[2];
        this->state_ = (uart_byte == frame_byte) ? ParseState::PARSE_STATE_HEADER_4 : ParseState::PARSE_STATE_HEADER_1;
        break;

      case ParseState::PARSE_STATE_HEADER_4:
        frame_byte = (this->frame_type_ == FrameType::FRAME_TYPE_COMMAND) ? CMD_HEADER[3] : REPORT_HEADER[3];
        this->state_ = (uart_byte == frame_byte) ? ParseState::PARSE_STATE_LEN_LOW : ParseState::PARSE_STATE_HEADER_1;
        break;

      case ParseState::PARSE_STATE_LEN_LOW:
        this->payload_len_ = uart_byte;
        this->state_ = ParseState::PARSE_STATE_LEN_HIGH;
        break;

      case ParseState::PARSE_STATE_LEN_HIGH:
        this->payload_len_ |= (static_cast<uint16_t>(uart_byte) << 8);
        this->payload_pos_ = 0;
        if (this->payload_len_ == 0) {
          // No target present - frame has no payload, go straight to footer.
          this->state_ = ParseState::PARSE_STATE_FOOTER_1;
        } else if (this->payload_len_ > LD2451_MAX_PAYLOAD_LEN) {
          // Sanity check - malformed/garbage length, resync.
          ESP_LOGW(TAG, "Frame length %u out of range, resyncing", this->payload_len_);
          this->state_ = ParseState::PARSE_STATE_HEADER_1;
        } else {
          this->state_ = ParseState::PARSE_STATE_PAYLOAD;
        }
        break;

      case ParseState::PARSE_STATE_PAYLOAD:
        this->payload_[this->payload_pos_++] = uart_byte;
        if (this->payload_pos_ >= this->payload_len_) {
          this->state_ = ParseState::PARSE_STATE_FOOTER_1;
        }
        break;

      case ParseState::PARSE_STATE_FOOTER_1:
        frame_byte = (this->frame_type_ == FrameType::FRAME_TYPE_COMMAND) ? CMD_FOOTER[0] : REPORT_FOOTER[0];
        this->state_ = (uart_byte == frame_byte) ? ParseState::PARSE_STATE_FOOTER_2 : ParseState::PARSE_STATE_HEADER_1;
        break;

      case ParseState::PARSE_STATE_FOOTER_2:
        frame_byte = (this->frame_type_ == FrameType::FRAME_TYPE_COMMAND) ? CMD_FOOTER[1] : REPORT_FOOTER[1];
        this->state_ = (uart_byte == frame_byte) ? ParseState::PARSE_STATE_FOOTER_3 : ParseState::PARSE_STATE_HEADER_1;
        break;

      case ParseState::PARSE_STATE_FOOTER_3:
        frame_byte = (this->frame_type_ == FrameType::FRAME_TYPE_COMMAND) ? CMD_FOOTER[2] : REPORT_FOOTER[2];
        this->state_ = (uart_byte == frame_byte) ? ParseState::PARSE_STATE_FOOTER_4 : ParseState::PARSE_STATE_HEADER_1;
        break;

      case ParseState::PARSE_STATE_FOOTER_4: {
        frame_byte = (this->frame_type_ == FrameType::FRAME_TYPE_COMMAND) ? CMD_FOOTER[3] : REPORT_FOOTER[3];
        this->state_ = ParseState::PARSE_STATE_HEADER_1;
        if (uart_byte != frame_byte) {
          ESP_LOGW(TAG, "Frame footer mismatch, dropping frame");
          break;
        }
        const bool is_command = this->frame_type_ == FrameType::FRAME_TYPE_COMMAND;
        char hex_buf[format_hex_pretty_size(LD2451_MAX_PAYLOAD_LEN)];
        ESP_LOGV(TAG, "%s frame: %s (%u)", is_command ? "Command" : "Report",
                 format_hex_pretty_to(hex_buf, this->payload_.data(), this->payload_pos_), this->payload_pos_);
        if (is_command) {
          this->handle_command_response_frame_(this->payload_.data(), this->payload_pos_);
        } else {
          this->handle_report_frame_(this->payload_.data(), this->payload_pos_);
        }
        break;
      }
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
  this->in_flight_cmd_ = command;
  if (value_len > 0 && value != nullptr) {
    this->write_array(value, value_len);
  }
  this->write_array(CMD_FOOTER, 4);
  this->flush();

  // Largest command value is 4 bytes (set target detection / sensitivity)
  char hex[format_hex_pretty_size(4)];
  ESP_LOGV(TAG, "Sent command: %02X, Data: %s (%d)", command, format_hex_pretty_to(hex, value, value_len), value_len);
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

// The SET commands write a whole block, so each field is the user's requested
// value if there is one, otherwise the value last read from the radar.
void LD2451Component::set_sensitivity_() {
  const uint8_t req = this->req_fields_;
  const uint8_t val[4] = {(req & CONFIG_FIELD_TRIGGER_COUNT) ? this->req_trigger_count_ : this->cfg_trigger_count_,
                          (req & CONFIG_FIELD_SNR_THRESHOLD) ? this->req_snr_threshold_ : this->cfg_snr_threshold_,
                          0x00, 0x00};
  this->sent_fields_ = req & SENSITIVITY_FIELDS;
  this->write_command_frame_(CMD_SET_SENSITIVITY, val, sizeof(val));
}

void LD2451Component::get_target_detection_cfg_() {
  this->write_command_frame_(CMD_GET_TARGET_DETECTION_CFG, nullptr, 0);
}

void LD2451Component::set_target_detection_cfg_() {
  const uint8_t req = this->req_fields_;
  const LD2451Direction direction = (req & CONFIG_FIELD_DIRECTION) ? this->req_direction_ : this->cfg_direction_;
  const uint8_t val[4] = {
      (req & CONFIG_FIELD_MAX_DISTANCE) ? this->req_max_distance_ : this->cfg_max_distance_,
      static_cast<uint8_t>(direction),
      (req & CONFIG_FIELD_MIN_SPEED) ? this->req_min_speed_ : this->cfg_min_speed_,
      (req & CONFIG_FIELD_NO_TARGET_DELAY) ? this->req_no_target_delay_ : this->cfg_no_target_delay_,
  };
  this->sent_fields_ = req & TARGET_DETECTION_FIELDS;
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
    case LD2451Direction::LD2451_DIRECTION_AWAY:
      return "Away";
    case LD2451Direction::LD2451_DIRECTION_TOWARD:
      return "Toward";
    case LD2451Direction::LD2451_DIRECTION_ALL:
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
                "  Trigger count: %u\n"
                "  Bluetooth: %s (assumed state - not read back from radar)",
                COMPONENT_VERSION, this->firmware_version_.empty() ? "Unknown" : this->firmware_version_.c_str(),
                this->comms_protocol_version_, this->cfg_max_distance_, this->cfg_min_speed_,
                this->cfg_no_target_delay_, direction_to_string(this->cfg_direction_), this->cfg_snr_threshold_,
                this->cfg_trigger_count_, ONOFF(this->cfg_bluetooth_enabled_));
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
      t.direction = base[2] == 0x01 ? LD2451Direction::LD2451_DIRECTION_TOWARD : LD2451Direction::LD2451_DIRECTION_AWAY;
      t.speed = base[3];
      t.snr = base[4];
      any_target = true;
      if (t.direction == LD2451Direction::LD2451_DIRECTION_TOWARD)
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
        float signed_speed = t.direction == LD2451Direction::LD2451_DIRECTION_TOWARD ? static_cast<float>(t.speed)
                                                                                     : -static_cast<float>(t.speed);
        this->target_speed_sensors_[i]->publish_state(signed_speed);
      } else if (!std::isnan(this->target_speed_sensors_[i]->get_raw_state())) {
        this->target_speed_sensors_[i]->publish_state(NAN);
      }
    }
#endif
#ifdef USE_BINARY_SENSOR
    if (this->target_approaching_binary_sensors_[i] != nullptr) {
      this->target_approaching_binary_sensors_[i]->publish_state(
          t.valid && t.direction == LD2451Direction::LD2451_DIRECTION_TOWARD);
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
// NOTE: Setters only record the requested value. A successful SET is
//       followed by a GET so the entities show the value from the LD2451.
// ---------------------------------------------------------------------

void LD2451Component::refresh_config() {
  this->pending_commands_ |= CommandFlags::COMMAND_FLAG_READ_FIRMWARE |
                             CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION |
                             CommandFlags::COMMAND_FLAG_GET_SENSITIVITY | CommandFlags::COMMAND_FLAG_DUMP_CONFIG;
}

void LD2451Component::request_config_(uint8_t field) {
  this->req_fields_ |= field;
  // A SET already in flight carries the old value of this field
  this->sent_fields_ &= ~field;
  if (field & TARGET_DETECTION_FIELDS) {
    this->pending_commands_ |= this->target_detection_read_ ? CommandFlags::COMMAND_FLAG_SET_TARGET_DETECTION
                                                            : CommandFlags::COMMAND_FLAG_GET_TARGET_DETECTION;
  } else {
    this->pending_commands_ |= this->sensitivity_read_ ? CommandFlags::COMMAND_FLAG_SET_SENSITIVITY
                                                       : CommandFlags::COMMAND_FLAG_GET_SENSITIVITY;
  }
}

void LD2451Component::config_written_(CommandFlags set_flag, CommandFlags get_flag, uint8_t fields) {
  this->req_fields_ &= ~(this->sent_fields_ & fields);
  this->sent_fields_ = 0;
  this->complete_command_(set_flag);
  this->pending_commands_ |= get_flag;
  // A field changed again while this SET was in flight
  if (this->req_fields_ & fields)
    this->pending_commands_ |= set_flag;
  this->command_state_ = CommandState::COMMAND_STATE_SEND_COMMAND;
}

void LD2451Component::set_max_distance(uint8_t meters) {
  if (meters < 10)
    meters = 10;
  if (meters > 100)
    meters = 100;
  this->req_max_distance_ = meters;
  this->request_config_(CONFIG_FIELD_MAX_DISTANCE);
}

void LD2451Component::set_min_speed(uint8_t kmh) {
  if (kmh > 0x78)
    kmh = 0x78;
  this->req_min_speed_ = kmh;
  this->request_config_(CONFIG_FIELD_MIN_SPEED);
}

void LD2451Component::set_no_target_delay(uint8_t seconds) {
  this->req_no_target_delay_ = seconds;
  this->request_config_(CONFIG_FIELD_NO_TARGET_DELAY);
}

void LD2451Component::set_detection_direction(LD2451Direction direction) {
  this->req_direction_ = direction;
  this->request_config_(CONFIG_FIELD_DIRECTION);
}

void LD2451Component::set_snr_threshold(uint8_t snr) {
  if (snr < 3)
    snr = 3;
  if (snr > 8)
    snr = 8;
  this->req_snr_threshold_ = snr;
  this->request_config_(CONFIG_FIELD_SNR_THRESHOLD);
}

void LD2451Component::set_trigger_count(uint8_t count) {
  if (count > 10)
    count = 10;
  this->req_trigger_count_ = count;
  this->request_config_(CONFIG_FIELD_TRIGGER_COUNT);
}

void LD2451Component::set_bluetooth_enable(bool enable) {
  ESP_LOGI(TAG, "Bluetooth %s - restarting module for it to take effect", ONOFF(enable));
  this->cfg_bluetooth_enabled_ = enable;
  this->pending_commands_ |= CommandFlags::COMMAND_FLAG_BLUETOOTH;
  this->pending_commands_ |= CommandFlags::COMMAND_FLAG_RESTART;
}

void LD2451Component::factory_reset() {
  // The reset only takes effect after a restart, so queue one too
  this->pending_commands_ |= CommandFlags::COMMAND_FLAG_FACTORY_RESET | CommandFlags::COMMAND_FLAG_RESTART;
  ESP_LOGI(TAG, "Factory reset requested - the module will restart");
}

void LD2451Component::restart_module() { this->pending_commands_ |= CommandFlags::COMMAND_FLAG_RESTART; }

}  // namespace esphome::ld2451
