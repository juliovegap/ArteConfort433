#pragma once

#include "esphome/core/defines.h"

// Only compiled when the `fan` component is part of the configuration; otherwise its
// headers are not even present in the build tree.
#ifdef USE_FAN

#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/fan/fan.h"

#include "arteconfort_433.h"

namespace esphome::arteconfort_433 {

/// Assumed-state fan entity on top of the one-way ArteConfort protocol.
///
/// It NEVER transmits on boot or when restoring the state. It also follows what the hub sends
/// through other paths (buttons, actions), so a "Velocidad 3" button keeps this entity in sync.
/// Presses on the physical remote cannot be observed.
class ArteConfort433Fan : public Component, public fan::Fan {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(ArteConfort433 *parent) { this->parent_ = parent; }
  void set_off_pattern(const char *pattern) { this->off_pattern_ = pattern; }
  void add_speed_pattern(const char *pattern) { this->speed_patterns_.push_back(pattern); }
  void set_breeze_pattern(const char *pattern) {
    this->breeze_pattern_ = pattern;
    // Registered at codegen time (like the official fans) so traits are valid before setup().
    this->set_supported_preset_modes({BREEZE_PRESET});
  }

  fan::FanTraits get_traits() override {
    this->wire_preset_modes_(this->traits_);
    return this->traits_;
  }

  static constexpr const char *BREEZE_PRESET = "Breeze";

 protected:
  void control(const fan::FanCall &call) override;
  void on_pattern_sent_(const char *pattern);

  ArteConfort433 *parent_{nullptr};
  const char *off_pattern_{nullptr};
  const char *breeze_pattern_{nullptr};
  std::vector<const char *> speed_patterns_;
  fan::FanTraits traits_;
  bool in_control_{false};
};

}  // namespace esphome::arteconfort_433

#endif  // USE_FAN
