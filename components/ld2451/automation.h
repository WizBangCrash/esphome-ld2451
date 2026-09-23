#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "ld2451.h"

namespace esphome::ld2451 {

template<typename... Ts>
class LD2451FactoryResetAction final : public Action<Ts...>, public Parented<LD2451Component> {
 public:
  void play(const Ts &...x) override { this->parent_->factory_reset(); }
};

template<typename... Ts>
class LD2451RestartAction final : public Action<Ts...>, public Parented<LD2451Component> {
 public:
  void play(const Ts &...x) override { this->parent_->restart_module(); }
};

template<typename... Ts>
class LD2451RefreshConfigAction final : public Action<Ts...>, public Parented<LD2451Component> {
 public:
  void play(const Ts &...x) override { this->parent_->refresh_config(); }
};

}  // namespace esphome::ld2451
