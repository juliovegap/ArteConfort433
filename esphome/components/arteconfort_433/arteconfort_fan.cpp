#include "arteconfort_fan.h"

#ifdef USE_FAN

#include <cstring>

#include "esphome/core/log.h"

namespace esphome::arteconfort_433 {

static const char *const TAG = "arteconfort_433.fan";

void ArteConfort433Fan::setup() {
  // No oscillation, speed supported, no direction.
  this->traits_ = fan::FanTraits(false, true, false, static_cast<int>(this->speed_patterns_.size()));

  // Restore the *assumed* state only. Nothing is transmitted: the physical fan
  // keeps its own state across ESP reboots.
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(*this);
  }

  if (this->parent_ != nullptr) {
    this->parent_->add_on_send_callback([this](const char *pattern) { this->on_pattern_sent_(pattern); });
  }
}

void ArteConfort433Fan::dump_config() {
  LOG_FAN("", "ArteConfort 433 Fan", this);
  ESP_LOGCONFIG(TAG,
                "  Speeds: %u\n"
                "  Off pattern: %s\n"
                "  Breeze pattern: %s",
                (unsigned) this->speed_patterns_.size(), this->off_pattern_ != nullptr ? this->off_pattern_ : "-",
                this->breeze_pattern_ != nullptr ? this->breeze_pattern_ : "-");
}

void ArteConfort433Fan::control(const fan::FanCall &call) {
  auto call_state = call.get_state();
  if (call_state.has_value())
    this->state = *call_state;
  auto call_speed = call.get_speed();
  if (call_speed.has_value())
    this->speed = *call_speed;
  this->apply_preset_mode_(call);

  if (this->parent_ != nullptr) {
    this->in_control_ = true;  // our own transmission must not echo back into on_pattern_sent_()
    if (!this->state) {
      this->parent_->send_pattern(this->off_pattern_);
    } else if (call.has_preset_mode() && this->breeze_pattern_ != nullptr) {
      this->parent_->send_pattern(this->breeze_pattern_);
    } else {
      int speed = this->speed;
      if (speed < 1)
        speed = 1;
      if (speed > static_cast<int>(this->speed_patterns_.size()))
        speed = static_cast<int>(this->speed_patterns_.size());
      this->speed = speed;
      this->parent_->send_pattern(this->speed_patterns_[speed - 1]);
    }
    this->in_control_ = false;
  }

  this->publish_state();
}

void ArteConfort433Fan::on_pattern_sent_(const char *pattern) {
  if (this->in_control_ || pattern == nullptr)
    return;

  if (this->off_pattern_ != nullptr && strcmp(pattern, this->off_pattern_) == 0) {
    this->state = false;
    this->publish_state();
    return;
  }
  if (this->breeze_pattern_ != nullptr && strcmp(pattern, this->breeze_pattern_) == 0) {
    this->state = true;
    this->set_preset_mode_(BREEZE_PRESET);
    this->publish_state();
    return;
  }
  for (size_t i = 0; i < this->speed_patterns_.size(); i++) {
    if (strcmp(pattern, this->speed_patterns_[i]) == 0) {
      this->state = true;
      this->speed = static_cast<int>(i) + 1;
      this->clear_preset_mode_();  // choosing a speed leaves any preset
      this->publish_state();
      return;
    }
  }
}

}  // namespace esphome::arteconfort_433

#endif  // USE_FAN
