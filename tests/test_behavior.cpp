// Behavior tests: hub + fan running on top of the real ESPHome core (host build),
// with a mock transmitter that records what would go on the air.
#include <cstdio>
#include <string>
#include <vector>

#include "esphome/core/application.h"
#include "esphome/components/arteconfort_433/arteconfort_433.h"
#include "esphome/components/arteconfort_433/arteconfort_fan.h"
#include "esphome/components/host/preferences.h"
#include "esphome/components/logger/logger.h"

using namespace esphome;
using namespace esphome::arteconfort_433;

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      failures++;                                         \
      std::printf("FAIL line %d: %s  ", __LINE__, #cond); \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

class MockTx : public remote_base::RemoteTransmitterBase {
 public:
  MockTx() : RemoteTransmitterBase(nullptr) {}
  struct Sent {
    remote_base::RawTimings data;
    uint32_t times;
    uint32_t wait;
    uint32_t carrier;
  };
  std::vector<Sent> sent;

 protected:
  void send_internal(uint32_t send_times, uint32_t send_wait) override {
    sent.push_back({this->temp_.get_data(), send_times, send_wait, this->temp_.get_carrier_frequency()});
  }
};

static remote_base::RawTimings expected_raw(uint64_t code, uint8_t bits = 30) {
  remote_base::RemoteTransmitData d;
  encode_frame(&d, FrameTiming{}, code, bits);
  return remote_base::RawTimings(d.get_data());
}

static const uint64_t OFF = 0x1EA0DD91ULL, S1 = 0x1EA0DDE8ULL, S2 = 0x1EA0DDC8ULL, S3 = 0x1EA0DDA9ULL, BREEZE = 0x1EA0DD0BULL;

static void setup_hub(ArteConfort433 &hub, MockTx *tx) {
  if (tx != nullptr)
    hub.set_transmitter(tx);
  hub.add_pattern("fanoff", OFF);
  hub.add_pattern("fanspeed1", S1);
  hub.add_pattern("fanspeed2", S2);
  hub.add_pattern("fanspeed3", S3);
  hub.add_pattern("breeze", BREEZE);
}

// The host platform's entry point (compiled with its main() renamed) references these.
void setup() {}
void loop() {}

int main() {
  // Same bring-up the generated main.cpp does for the host platform.
  auto *logger = new logger::Logger(115200);
  logger->create_pthread_key();
  logger->pre_setup();
  logger->set_log_level(ESPHOME_LOG_LEVEL_WARN);  // keep the test output quiet

  new (&App) Application();
  App.pre_setup("test", 4, "", 0);
  host::setup_preferences();

  // ---------------------------------------------------------------- hub
  {
    MockTx tx;
    ArteConfort433 hub;
    setup_hub(hub, &tx);

    CHECK(hub.has_pattern("fanoff") && !hub.has_pattern("nope") && !hub.has_pattern(nullptr), "has_pattern");

    CHECK(hub.send_pattern("fanoff"), "send known pattern");
    CHECK(tx.sent.size() == 1, "one transmission, got %zu", tx.sent.size());
    if (tx.sent.size() == 1) {
      CHECK(tx.sent[0].times == 4 && tx.sent[0].wait == 5000, "defaults repeat=4 gap=5000 (got %u/%u)", tx.sent[0].times, tx.sent[0].wait);
      CHECK(tx.sent[0].carrier == 0, "pure OOK: no software carrier");
      CHECK(tx.sent[0].data == expected_raw(OFF), "waveform of fanoff");
    }

    CHECK(!hub.send_pattern("nope") && !hub.send_pattern(nullptr), "unknown/null pattern rejected");
    CHECK(tx.sent.size() == 1, "rejected patterns transmit nothing");

    CHECK(!hub.send_code(1ULL << 30, 30), "code that does not fit 30 bits rejected");
    CHECK(!hub.send_code(1, 65), "65 bits rejected");
    CHECK(tx.sent.size() == 1, "invalid codes transmit nothing");

    CHECK(hub.send_code(S3, 0, 2), "send_code with repeat override");
    CHECK(tx.sent.size() == 2 && tx.sent[1].times == 2 && tx.sent[1].data == expected_raw(S3), "repeat override applied");
    CHECK(hub.send_code(0xFFFF, 16), "send_code with bits override");
    CHECK(tx.sent.size() == 3 && tx.sent[2].data == expected_raw(0xFFFF, 16), "bits override applied");

    ArteConfort433 no_tx;
    setup_hub(no_tx, nullptr);
    CHECK(!no_tx.send_pattern("fanoff"), "no transmitter -> false, no crash");
  }

  // ------------------------------------------------------------ callbacks
  {
    MockTx tx;
    ArteConfort433 hub;
    setup_hub(hub, &tx);
    std::vector<std::string> names;
    hub.add_on_send_callback([&](const char *n) { names.push_back(n); });

    hub.send_pattern("fanspeed1");
    hub.send_code(S2);                  // raw code that matches a pattern
    hub.send_code(0x12345);             // unknown code
    hub.send_code(S3, 24);              // known value but different bit count
    CHECK(names.size() == 2 && names[0] == "fanspeed1" && names[1] == "fanspeed2", "callbacks: got %zu", names.size());
  }

  // ---------------------------------------------------------------- fan
  {
    MockTx tx;
    ArteConfort433 hub;
    setup_hub(hub, &tx);

    ArteConfort433Fan fan;
    fan.set_parent(&hub);
    fan.set_off_pattern("fanoff");
    fan.add_speed_pattern("fanspeed1");
    fan.add_speed_pattern("fanspeed2");
    fan.add_speed_pattern("fanspeed3");
    fan.set_breeze_pattern("breeze");
    fan.set_restore_mode(fan::FanRestoreMode::RESTORE_DEFAULT_OFF);
    App.register_fan(&fan, "Ventilador", 4242, 0);

    int publishes = 0;
    fan.add_on_state_callback([&]() { publishes++; });

    fan.setup();
    CHECK(tx.sent.empty(), "setup()/restore must NEVER transmit");
    publishes = 0;  // restoring the state publishes it once (ESPHome's FanRestoreState::apply)
    CHECK(!fan.state, "boots off");
    CHECK(fan.get_traits().supported_speed_count() == 3, "3 speeds");
    CHECK(fan.get_traits().supports_preset_modes(), "breeze preset registered before setup");

    // Controlled from the fan entity.
    fan.turn_on().set_speed(2).perform();
    CHECK(fan.state && fan.speed == 2, "on at speed 2");
    CHECK(tx.sent.size() == 1 && tx.sent[0].data == expected_raw(S2), "sent fanspeed2");
    CHECK(publishes == 1, "own command publishes once, got %d (no echo)", publishes);

    fan.make_call().set_speed(3).perform();
    CHECK(fan.speed == 3 && tx.sent.size() == 2 && tx.sent[1].data == expected_raw(S3), "speed change sends fanspeed3");

    fan.turn_off().perform();
    CHECK(!fan.state && tx.sent.size() == 3 && tx.sent[2].data == expected_raw(OFF), "off sends fanoff");

    fan.turn_on().perform();  // remembers the last speed
    CHECK(fan.state && fan.speed == 3 && tx.sent.size() == 4 && tx.sent[3].data == expected_raw(S3), "turn on resumes speed 3");

    fan.make_call().set_preset_mode("Breeze").perform();
    CHECK(fan.has_preset_mode() && tx.sent.size() == 5 && tx.sent[4].data == expected_raw(BREEZE), "breeze preset sends breeze");
    fan.make_call().set_speed(1).perform();
    CHECK(!fan.has_preset_mode() && fan.speed == 1 && tx.sent.size() == 6 && tx.sent[5].data == expected_raw(S1),
          "choosing a speed leaves the preset");

    // Driven from outside (a button / action) -> entity follows, without transmitting again.
    const size_t before = tx.sent.size();
    hub.send_pattern("fanspeed2");
    CHECK(tx.sent.size() == before + 1, "button transmits exactly once");
    CHECK(fan.state && fan.speed == 2 && !fan.has_preset_mode(), "fan follows button: speed 2");
    hub.send_pattern("fanoff");
    CHECK(!fan.state, "fan follows button: off");
    hub.send_pattern("breeze");
    CHECK(fan.state && fan.has_preset_mode(), "fan follows button: breeze");
    hub.send_pattern("fanspeed3");
    CHECK(fan.state && fan.speed == 3 && !fan.has_preset_mode(), "fan follows button: speed 3 clears preset");
  }

  // ------------------------------------------------------------ receiver
  {
    ArteConfort433 hub;
    setup_hub(hub, nullptr);
    hub.set_receiving(true);
    remote_base::RawTimings good = expected_raw(S1);
    good.push_back(-10000);
    CHECK(!hub.on_receive(remote_base::RemoteReceiveData(good, 25, remote_base::TOLERANCE_MODE_PERCENTAGE)),
          "on_receive never swallows the frame (raw dumper still sees it)");
    remote_base::RawTimings junk{300, -400, 500, -200};
    CHECK(!hub.on_receive(remote_base::RemoteReceiveData(junk, 25, remote_base::TOLERANCE_MODE_PERCENTAGE)), "junk is ignored");
    // A glued train of repetitions (as captured from the real remote) must be accepted too.
    remote_base::RawTimings train;
    for (int k = 0; k < 4; k++) {
      auto f = expected_raw(S1);
      train.insert(train.end(), f.begin(), f.end());
    }
    CHECK(!hub.on_receive(remote_base::RemoteReceiveData(train, 25, remote_base::TOLERANCE_MODE_PERCENTAGE)), "train handled");
  }

  // ------------------------------------------------- Sulion preset + auto-detect
  {
    // Same values the Python codegen applies for `protocol: sulion`.
    MockTx tx;
    ArteConfort433 hub;
    hub.set_transmitter(&tx);
    hub.set_protocol("sulion");
    hub.set_bits(32);
    hub.set_gap_us(12000);
    hub.set_short_us(380);
    hub.set_long_us(1240);
    hub.set_one_mark_us(1190);
    hub.set_one_space_us(430);
    hub.set_zero_mark_us(380);
    hub.set_zero_space_us(1240);
    hub.set_trailer_mark_us(380);
    hub.add_pattern("key_a", 0xCA3804F6ULL);
    CHECK(hub.send_pattern("key_a"), "sulion send");
    CHECK(tx.sent.size() == 1 && tx.sent[0].wait == 12000 && tx.sent[0].data.size() == 65, "sulion: 65 elements, 12 ms gap");
    if (tx.sent.size() == 1) {
      CHECK(tx.sent[0].data[0] == 1190 && tx.sent[0].data[1] == -430, "bit 1 = 1190/430 (0xC = 1100...)");
      CHECK(tx.sent[0].data[4] == 380 && tx.sent[0].data[5] == -1240, "bit 0 = 380/1240");
      CHECK(tx.sent[0].data[64] == 380, "trailing mark 380");
    }
    CHECK(!hub.send_code(1ULL << 32, 32), "sulion: code wider than 32 bits rejected");

    // Receive-only hub in auto mode decodes both protocols.
    ArteConfort433 rx;
    rx.set_receiving(true);
    rx.set_auto_detect(true);
    rx.set_protocol("auto");
    remote_base::RawTimings sul{1190,-440,1180,-420,383,-1248,387,-1227,1189,-432,388,-1230,1191,-429,361,-1251,390,-1232,387,-1231,1190,-430,1191,-428,1191,-431,386,-1229,389,-1229,388,-1228,391,-1233,390,-1228,388,-1227,390,-1229,389,-1233,1192,-431,389,-1229,389,-1229,1192,-435,1193,-430,1187,-428,1188,-429,389,-1233,1193,-430,1192,-429,387,-1228,389};
    rx.on_receive(remote_base::RemoteReceiveData(sul, 25, remote_base::TOLERANCE_MODE_PERCENTAGE));
    CHECK(rx.last_protocol() != nullptr && std::string(rx.last_protocol()) == "sulion" && rx.last_analysis().code == 0xCA3804F6ULL,
          "auto: sulion detected");
    // An ArteConfort frame as the receiver delivers it: [mark][gap][29 bits].
    remote_base::RawTimings arte = expected_raw(0x3D41BBA1ULL, 30);
    arte.back() -= 5000;
    remote_base::RawTimings piece(arte.end() - 2, arte.end());
    piece.insert(piece.end(), arte.begin(), arte.end() - 2);
    rx.on_receive(remote_base::RemoteReceiveData(piece, 25, remote_base::TOLERANCE_MODE_PERCENTAGE));
    CHECK(rx.last_protocol() != nullptr && std::string(rx.last_protocol()) == "arteconfort" && rx.last_analysis().code == 0x3D41BBA1ULL,
          "auto: arteconfort detected (got %s 0x%llX)", rx.last_protocol() ? rx.last_protocol() : "none",
          (unsigned long long) rx.last_analysis().code);
  }


  // Real capture of the 1 h key: a leading lone pair (mark ~390 + gap ~5738) left over from the receiver restarting,
  // then complete frames ending in a long mark (bit 1). The code is 0x3D41B92B, never ...2A.
  {
    ArteConfort433 rx;
    rx.set_receiving(true);
    remote_base::RawTimings real{399,-5723,1128,-364,1128,-369,1153,-363,1125,-369,395,-1117,1142,-374,383,-1123,1121,-370,380,-1147,368,-1115,383,-1124,401,-1117,383,-1110,1149,-365,1142,-371,366,-1126,1148,-370,1135,-367,1129,-364,373,-1147,374,-1127,1122,-371,397,-1100,389,-1117,1143,-368,390,-1116,1135,-353,393,-1119,1150,-351,1142,-5386,1157,-368,1132,-358,1138,-370,1141,-366,370,-1150,1132,-342,403,-1124,1123,-371,383,-1123,391,-1115,382,-1122,391,-1115,382,-1123,1143,-350,1138,-374,390,-1112,1130,-385,1124,-371,1134,-382,370,-1117,401,-1124,1121,-371,383,-1122,390,-1113,1133,-366,401,-1115,1134,-370,383,-1121,1125,-364,1156};
    rx.on_receive(remote_base::RemoteReceiveData(real, 25, remote_base::TOLERANCE_MODE_PERCENTAGE));
    CHECK(rx.last_analysis().code == 0x3D41B92BULL, "real 1h reception decodes 0x3D41B92B (got 0x%llX)",
          (unsigned long long) rx.last_analysis().code);
  }

  std::printf(failures ? "\n%d FAILURE(S)\n" : "\nALL BEHAVIOR TESTS PASSED\n", failures);
  return failures ? 1 : 0;
}
