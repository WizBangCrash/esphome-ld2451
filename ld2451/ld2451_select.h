#pragma once

#include "esphome/components/select/select.h"
#include "ld2451.h"

namespace esphome {
namespace ld2451 {

class LD2451DirectionSelect : public select::Select, public Component {
 public:
  void set_parent(LD2451Component *parent) { this->parent_ = parent; }

  void setup() override {
    switch (this->parent_->get_config_direction()) {
      case LD2451Direction::AWAY:
        this->publish_state("AWAY");
        break;
      case LD2451Direction::TOWARD:
        this->publish_state("TOWARD");
        break;
      case LD2451Direction::ALL:
      default:
        this->publish_state("ALL");
        break;
    }
  }

 protected:
  void control(const std::string &value) override {
    LD2451Direction dir = LD2451Direction::ALL;
    if (value == "AWAY") {
      dir = LD2451Direction::AWAY;
    } else if (value == "TOWARD") {
      dir = LD2451Direction::TOWARD;
    }
    this->parent_->set_detection_direction(dir);
    this->publish_state(value);
  }

  LD2451Component *parent_{nullptr};
};

}  // namespace ld2451
}  // namespace esphome
