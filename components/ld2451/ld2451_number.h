#pragma once

#include "esphome/components/number/number.h"
#include "ld2451.h"

namespace esphome {
namespace ld2451 {

enum class LD2451NumberField : uint8_t {
  LD2451_NUMBER_FIELD_MAX_DISTANCE,
  LD2451_NUMBER_FIELD_MIN_SPEED,
  LD2451_NUMBER_FIELD_NO_TARGET_DELAY,
  LD2451_NUMBER_FIELD_SNR_THRESHOLD,
  LD2451_NUMBER_FIELD_TRIGGER_COUNT,
};

class LD2451Number : public number::Number, public Component {
 public:
  void set_parent(LD2451Component *parent) { this->parent_ = parent; }
  void set_field(LD2451NumberField field) { this->field_ = field; }

  void setup() override {
    switch (this->field_) {
      case LD2451NumberField::LD2451_NUMBER_FIELD_MAX_DISTANCE:
        this->publish_state(this->parent_->get_config_max_distance());
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_MIN_SPEED:
        this->publish_state(this->parent_->get_config_min_speed());
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_NO_TARGET_DELAY:
        this->publish_state(this->parent_->get_config_no_target_delay());
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_SNR_THRESHOLD:
        this->publish_state(this->parent_->get_config_snr_threshold());
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_TRIGGER_COUNT:
        this->publish_state(this->parent_->get_config_trigger_count());
        break;
    }
  }

 protected:
  void control(float value) override {
    uint8_t v = static_cast<uint8_t>(value);
    switch (this->field_) {
      case LD2451NumberField::LD2451_NUMBER_FIELD_MAX_DISTANCE:
        this->parent_->set_max_distance(v);
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_MIN_SPEED:
        this->parent_->set_min_speed(v);
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_NO_TARGET_DELAY:
        this->parent_->set_no_target_delay(v);
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_SNR_THRESHOLD:
        this->parent_->set_snr_threshold(v);
        break;
      case LD2451NumberField::LD2451_NUMBER_FIELD_TRIGGER_COUNT:
        this->parent_->set_trigger_count(v);
        break;
    }
    this->publish_state(value);
  }

  LD2451Component *parent_{nullptr};
  LD2451NumberField field_{LD2451NumberField::LD2451_NUMBER_FIELD_MAX_DISTANCE};
};

}  // namespace ld2451
}  // namespace esphome
