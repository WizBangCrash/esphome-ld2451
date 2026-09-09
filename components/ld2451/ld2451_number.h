#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/optional.h"
#include "ld2451.h"

namespace esphome {
namespace ld2451 {

enum class LD2451NumberField : uint8_t {
  MAX_DISTANCE,
  MIN_SPEED,
  NO_TARGET_DELAY,
  SNR_THRESHOLD,
};

class LD2451Number : public number::Number, public Component {
 public:
  void set_parent(LD2451Component *parent) { this->parent_ = parent; }
  void set_field(LD2451NumberField field) { this->field_ = field; }
  void set_default_value(uint8_t v) { this->default_value_ = v; }

  void setup() override {
    if (this->default_value_.has_value()) {
      this->control(*this->default_value_);
      return;
    }
    switch (this->field_) {
      case LD2451NumberField::MAX_DISTANCE:
        this->publish_state(this->parent_->get_config_max_distance());
        break;
      case LD2451NumberField::MIN_SPEED:
        this->publish_state(this->parent_->get_config_min_speed());
        break;
      case LD2451NumberField::NO_TARGET_DELAY:
        this->publish_state(this->parent_->get_config_no_target_delay());
        break;
      case LD2451NumberField::SNR_THRESHOLD:
        this->publish_state(this->parent_->get_config_snr_threshold());
        break;
    }
  }

 protected:
  void control(float value) override {
    uint8_t v = static_cast<uint8_t>(value);
    switch (this->field_) {
      case LD2451NumberField::MAX_DISTANCE:
        this->parent_->set_max_distance(v);
        break;
      case LD2451NumberField::MIN_SPEED:
        this->parent_->set_min_speed(v);
        break;
      case LD2451NumberField::NO_TARGET_DELAY:
        this->parent_->set_no_target_delay(v);
        break;
      case LD2451NumberField::SNR_THRESHOLD:
        this->parent_->set_snr_threshold(v);
        break;
    }
    this->publish_state(value);
  }

  LD2451Component *parent_{nullptr};
  LD2451NumberField field_{LD2451NumberField::MAX_DISTANCE};
  optional<uint8_t> default_value_{};
};

}  // namespace ld2451
}  // namespace esphome
