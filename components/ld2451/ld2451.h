#pragma once

#include <vector>
#include <string>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {

// Keep the pointer declarations valid even when the sensor platform header is
// not enabled in this translation unit.
namespace sensor {
class Sensor;
}

namespace ld2451 {

// LD2451 supports up to 5 simultaneously tracked vehicle/pedestrian targets.
static const uint8_t LD2451_MAX_TARGETS = 5;

enum class LD2451Direction : uint8_t {
  AWAY = 0x00,     // target moving away from the radar
  TOWARD = 0x01,   // target moving toward the radar
  ALL = 0x02,      // used only for the "detection direction" config parameter
};

struct LD2451Target {
  bool valid{false};
  int8_t angle{0};        // degrees, signed (raw byte - 0x80)
  uint8_t distance{0};    // meters
  LD2451Direction direction{LD2451Direction::AWAY};
  uint8_t speed{0};       // km/h
  uint8_t snr{0};         // signal to noise ratio, 0-255
};

class LD2451Component : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ---- Configuration (called from number/select/switch/button platforms) ----
  // These issue a full enable-config / command / end-config exchange with the
  // radar. They are blocking (bounded by COMMAND_TIMEOUT_MS) and are intended
  // to be called rarely (e.g. from a UI control), never from a fast loop.
  void set_max_distance(uint8_t meters);        // 10-100 m
  void set_min_speed(uint8_t kmh);               // 0-120 km/h
  void set_no_target_delay(uint8_t seconds);     // 0-255 s
  void set_detection_direction(LD2451Direction direction);
  void set_snr_threshold(uint8_t snr);           // 3-8
  void set_multi_trigger(bool require_multiple);  // "cumulative effective trigger times"
  // Enables/disables the radar's built-in Bluetooth (see the CMD_BLUETOOTH
  // note in ld2451.cpp - command word inferred from the LD2410, not
  // confirmed against an LD2451-specific datasheet excerpt). There is no
  // known query command to read this back from the radar, so
  // get_config_bluetooth_enabled() reflects only what this component has
  // itself set/assumed, not necessarily the radar's actual state at boot.
  void set_bluetooth_enabled(bool enable);
  void factory_reset();
  void restart_module();

  // Re-reads the current target-detection + sensitivity config from the
  // radar and pushes the values into any linked number/select/switch
  // entities. Safe to call from setup() or from a button/service call.
  void refresh_config();

#ifdef USE_SENSOR
  void set_target_count_sensor(sensor::Sensor *s) { target_count_sensor_ = s; }
  void set_target_angle_sensor(uint8_t idx, sensor::Sensor *s) { target_angle_sensors_[idx] = s; }
  void set_target_distance_sensor(uint8_t idx, sensor::Sensor *s) { target_distance_sensors_[idx] = s; }
  void set_target_speed_sensor(uint8_t idx, sensor::Sensor *s) { target_speed_sensors_[idx] = s; }
#endif

#ifdef USE_BINARY_SENSOR
  void set_has_target_binary_sensor(binary_sensor::BinarySensor *s) { has_target_binary_sensor_ = s; }
  void set_has_approaching_target_binary_sensor(binary_sensor::BinarySensor *s) {
    has_approaching_target_binary_sensor_ = s;
  }
  // true while target[idx] is present AND moving toward the radar
  void set_target_approaching_binary_sensor(uint8_t idx, binary_sensor::BinarySensor *s) {
    target_approaching_binary_sensors_[idx] = s;
  }
#endif

#ifdef USE_TEXT_SENSOR
  void set_firmware_version_text_sensor(text_sensor::TextSensor *s) { firmware_version_text_sensor_ = s; }
#endif

  // Used by number/select/switch platforms to know the config values last
  // read from the radar (populated after refresh_config() / setup()).
  uint8_t get_config_max_distance() const { return cfg_max_distance_; }
  uint8_t get_config_min_speed() const { return cfg_min_speed_; }
  uint8_t get_config_no_target_delay() const { return cfg_no_target_delay_; }
  LD2451Direction get_config_direction() const { return cfg_direction_; }
  uint8_t get_config_snr_threshold() const { return cfg_snr_threshold_; }
  bool get_config_multi_trigger() const { return cfg_multi_trigger_; }
  ::std::string get_firmware_version() const { return firmware_version_; }
  bool get_config_bluetooth_enabled() const { return cfg_bluetooth_enabled_; }

 protected:
  static const uint32_t COMMAND_TIMEOUT_MS = 500;
  // How long to wait, with no report frame received at all, before treating
  // the radar's silence as "no target present" (see loop()). Some units
  // stop transmitting entirely rather than sending an explicit zero-length
  // report frame when no target is in view.
  static const uint32_t IDLE_TIMEOUT_MS = 1500;

  // ---- low level protocol helpers ----
  bool write_command_frame_(uint8_t command, const uint8_t *value, uint8_t value_len);
  // Reads one ACK frame for `command`, waiting up to COMMAND_TIMEOUT_MS.
  // On success, `out` holds the ACK payload *after* the 2-byte status word
  // (i.e. status is checked here, out contains only extra returned data).
  bool read_ack_frame_(uint8_t command, std::vector<uint8_t> &out);
  bool send_command_(uint8_t command, const uint8_t *value, uint8_t value_len, std::vector<uint8_t> &response);
  bool enable_config_();
  bool end_config_();
  void drain_rx_();
  // Issues the 0xA0 read-firmware command and, on success, updates
  // firmware_version_ (and the linked text_sensor, if any).
  void query_firmware_version_();

  // ---- report-frame streaming parser (runs continuously in loop()) ----
  void process_byte_(uint8_t b);
  void handle_report_payload_(const uint8_t *data, uint16_t len);
  // Publishes target/count/presence state for `count` targets described by
  // `data` (or count=0, data=nullptr to publish an all-clear state). Shared
  // by handle_report_payload_() (actual frames) and clear_all_targets_()
  // (idle timeout fallback, see loop()).
  void publish_targets_(const uint8_t *data, uint8_t count);
  void clear_all_targets_();

  enum class ParseState : uint8_t {
    HEADER_1,  // 0xF4
    HEADER_2,  // 0xF3
    HEADER_3,  // 0xF2
    HEADER_4,  // 0xF1
    LEN_LOW,
    LEN_HIGH,
    PAYLOAD,
    FOOTER_1,  // 0xF8
    FOOTER_2,  // 0xF7
    FOOTER_3,  // 0xF6
    FOOTER_4,  // 0xF5
  };

  ParseState state_{ParseState::HEADER_1};
  uint16_t payload_len_{0};
  ::std::vector<uint8_t> payload_;

  // 0 = no report frame seen yet since boot (don't idle-clear before the
  // radar has said anything at all).
  uint32_t last_report_ms_{0};
  bool idle_cleared_{false};

  LD2451Target targets_[LD2451_MAX_TARGETS];

  // last-known config, populated by refresh_config()
  uint8_t cfg_max_distance_{100};
  uint8_t cfg_min_speed_{0};
  uint8_t cfg_no_target_delay_{0};
  LD2451Direction cfg_direction_{LD2451Direction::ALL};
  uint8_t cfg_snr_threshold_{4};
  bool cfg_multi_trigger_{false};
  // Assumed true (radar ships with Bluetooth on by default per the user
  // manual) - not read back from the radar since there's no known query
  // command for it. Only reflects what this component has itself set.
  bool cfg_bluetooth_enabled_{true};
  ::std::string firmware_version_{};

#ifdef USE_SENSOR
  sensor::Sensor *target_count_sensor_{nullptr};
  sensor::Sensor *target_angle_sensors_[LD2451_MAX_TARGETS]{};
  sensor::Sensor *target_distance_sensors_[LD2451_MAX_TARGETS]{};
  sensor::Sensor *target_speed_sensors_[LD2451_MAX_TARGETS]{};
#endif

#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *has_target_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *has_approaching_target_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *target_approaching_binary_sensors_[LD2451_MAX_TARGETS]{};
#endif

#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *firmware_version_text_sensor_{nullptr};
#endif
};

}  // namespace ld2451
}  // namespace esphome

// These are header-only platform classes. They live in subdirectories for
// organization, but must be pulled into any translation unit (i.e. the
// generated main.cpp) that references them - just being compiled as their
// own source file is not enough for a header-only class to be *defined*
// anywhere. USE_xxx are defined automatically by ESPHome when the
// corresponding platform (number:/select:/switch:/button:) is used.
#ifdef USE_NUMBER
#include "ld2451_number.h"
#endif
#ifdef USE_SELECT
#include "ld2451_select.h"
#endif
#ifdef USE_SWITCH
#include "ld2451_switch.h"
#endif
#ifdef USE_BUTTON
#include "ld2451_button.h"
#endif
