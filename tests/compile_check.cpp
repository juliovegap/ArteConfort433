// Compile-only check: instantiates the actions the way ESPHome's codegen emits them, and the
// platform classes only when their component is part of the configuration (USE_BUTTON / USE_FAN).
#include "esphome/components/arteconfort_433/arteconfort_433.h"
#ifdef USE_FAN
#include "esphome/components/arteconfort_433/arteconfort_fan.h"
#endif

using namespace esphome::arteconfort_433;

void instantiate_everything(ArteConfort433 *hub) {
  SendPatternAction<> a;
  a.set_parent(hub);
  a.set_pattern(ESPHOME_F("fanspeed2"));
  a.play();

  SendPatternAction<> a2;
  a2.set_parent(hub);
  a2.set_pattern([]() -> std::string { return "fanoff"; });
  a2.play();

  SendPatternAction<int> a3;
  a3.set_parent(hub);
  a3.set_pattern([](int) -> std::string { return "fanspeed1"; });
  a3.play(1);

  SendCodeAction<> c;
  c.set_parent(hub);
  c.set_code([]() -> uint64_t { return 513858961; });
  c.set_bits([]() -> uint8_t { return 29; });
  c.set_repeat([]() -> uint8_t { return 2; });
  c.play();

  SendCodeAction<> c2;
  c2.set_parent(hub);
  c2.set_code([]() -> uint64_t { return 0x1EA0DDE8ULL; });
  c2.play();

  hub->add_on_send_callback([](const char *) {});
  hub->set_receiving(true);

#ifdef USE_BUTTON
  ArteConfort433Button b;
  b.set_parent(hub);
  b.set_pattern("fanoff");
#endif
#ifdef USE_FAN
  ArteConfort433Fan f;
  f.set_parent(hub);
  f.add_speed_pattern("fanspeed1");
  f.set_off_pattern("fanoff");
  f.set_breeze_pattern("breeze");
#endif
}
