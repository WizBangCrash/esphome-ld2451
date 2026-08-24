#pragma once

#include "esphome/components/button/button.h"
#include "ld2451.h"

namespace esphome {
namespace ld2451 {

enum class LD2451ButtonAction : uint8_t {
  FACTORY_RESET,
  RESTART,
  REFRESH_CONFIG,
};

class LD2451Button : public button::Button, public Component {
 public:
  void set_parent(LD2451Component *parent) { this->parent_ = parent; }
  void set_action(LD2451ButtonAction action) { this->action_ = action; }

 protected:
  void press_action() override {
    switch (this->action_) {
      case LD2451ButtonAction::FACTORY_RESET:
        this->parent_->factory_reset();
        break;
      case LD2451ButtonAction::RESTART:
        this->parent_->restart_module();
        break;
      case LD2451ButtonAction::REFRESH_CONFIG:
        this->parent_->refresh_config();
        break;
    }
  }

  LD2451Component *parent_{nullptr};
  LD2451ButtonAction action_{LD2451ButtonAction::REFRESH_CONFIG};
};

}  // namespace ld2451
}  // namespace esphome
