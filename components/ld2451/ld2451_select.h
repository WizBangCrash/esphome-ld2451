#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/log.h"
#include "ld2451.h"

namespace esphome {
namespace ld2451 {

class LD2451DirectionSelect : public select::Select, public Component {
 public:
  void set_parent(LD2451Component *parent) { this->parent_ = parent; }

  void setup() override { this->publish_direction(this->parent_->get_config_direction()); }

  // Publishes the select's displayed state from an authoritative
  // LD2451Direction value (called from setup() and whenever the parent
  // re-reads the config from the radar). The option index equals the
  // LD2451Direction value - see OPTIONS in select.py.
  void publish_direction(LD2451Direction direction) {
    size_t index = static_cast<size_t>(direction);
    if (!this->has_index(index)) {
      ESP_LOGW("ld2451.select", "Unknown detection direction %u", static_cast<uint8_t>(direction));
      index = static_cast<size_t>(LD2451Direction::LD2451_DIRECTION_ALL);
    }
    this->publish_state(index);
  }

 protected:
  void control(size_t index) override {
    this->parent_->set_detection_direction(static_cast<LD2451Direction>(index));
    this->publish_state(index);
  }

  LD2451Component *parent_{nullptr};
};

}  // namespace ld2451
}  // namespace esphome
