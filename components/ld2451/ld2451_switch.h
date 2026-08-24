#pragma once

#include "esphome/components/switch/switch.h"
#include "ld2451.h"

namespace esphome {
namespace ld2451 {

class LD2451MultiTriggerSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(LD2451Component *parent) { this->parent_ = parent; }

  void setup() override { this->publish_state(this->parent_->get_config_multi_trigger()); }

 protected:
  void write_state(bool state) override {
    this->parent_->set_multi_trigger(state);
    this->publish_state(state);
  }

  LD2451Component *parent_{nullptr};
};

class LD2451BluetoothSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(LD2451Component *parent) { this->parent_ = parent; }

  // Note: this reflects only what the component has itself set/assumed
  // (defaults to "on", since the radar ships with Bluetooth enabled), not
  // a value read back from the radar - there's no known query command for
  // it. If your radar's actual state differs after a manual reset via the
  // app or a factory reset, this switch may be out of sync until toggled.
  void setup() override { this->publish_state(this->parent_->get_config_bluetooth_enabled()); }

 protected:
  void write_state(bool state) override {
    this->parent_->set_bluetooth_enabled(state);
    this->publish_state(state);
  }

  LD2451Component *parent_{nullptr};
};

}  // namespace ld2451
}  // namespace esphome
