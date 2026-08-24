#pragma once

#include "esphome/core/automation.h"
#include "ld2451.h"

namespace esphome {
namespace ld2451 {

template<typename... Ts> class LD2451FactoryResetAction : public Action<Ts...> {
 public:
  explicit LD2451FactoryResetAction(LD2451Component *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->factory_reset(); }

 protected:
  LD2451Component *parent_;
};

template<typename... Ts> class LD2451RestartAction : public Action<Ts...> {
 public:
  explicit LD2451RestartAction(LD2451Component *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->restart_module(); }

 protected:
  LD2451Component *parent_;
};

template<typename... Ts> class LD2451RefreshConfigAction : public Action<Ts...> {
 public:
  explicit LD2451RefreshConfigAction(LD2451Component *parent) : parent_(parent) {}
  void play(Ts... x) override { this->parent_->refresh_config(); }

 protected:
  LD2451Component *parent_;
};

}  // namespace ld2451
}  // namespace esphome
