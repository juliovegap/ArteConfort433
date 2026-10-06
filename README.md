# ArteConfort 433 - ESPHome Component

Control **ArteConfort** ceiling fans (OOK, 30-bit frames) with a CC1101 radio. **One ESP32 / ESP8266 with a
single CC1101 can drive several fans** (three are set up out of the box): each fan appears in Home Assistant as
its own device, in its own area, with a fan entity and 18 buttons.

The component contains **no radio driver**. It only encodes/decodes the protocol and delegates the rest to
official ESPHome components:

```
buttons / fan / actions ─► arteconfort_433 (one hub per fan) ─► remote_transmitter ─► GDO0 ─► CC1101 (cc1101)
```

No RadioLib, no `SPI.h` patches, no generated files in your config directory; works with ESP-IDF and Arduino.

## Requirements
ESPHome with the official `cc1101` component (validated with **2026.9.1**; the minimum version is unknown), ESP32 or
ESP8266, a 433 MHz CC1101 module (a 17.3 cm wire antenna on the ANT pin is recommended).

## Wiring
| CC1101 | ESP32  | NodeMCU (ESP8266) |
|--------|--------|-------------------|
| SCK    | GPIO18 | D5 / GPIO14       |
| MISO   | GPIO19 | D6 / GPIO12       |
| MOSI   | GPIO23 | D7 / GPIO13       |
| CSn    | GPIO5  | D8 / GPIO15       |
| GDO0   | GPIO4  | D1 / GPIO5        |

Recommended: a 330 ohm - 1 kohm series resistor on GDO0. On the ESP8266, GPIO15 must be LOW at boot.

## Quick start
1. Copy `esphome/` to your ESPHome configuration and create `secrets.yaml` (`wifi_ssid`, `wifi_password`).
2. Learn the codes of each remote with `arteconfort_capture_rx.yaml` (see below) and edit
   `patterns/living_room.yaml`, `patterns/bedroom.yaml` and `patterns/office.yaml`. The values shipped there are
   empty as **every value differ from remote to remote**.
3. Build `arteconfort_esp32.yaml` (or `arteconfort_esp8266.yaml`).

Every room is one block under `packages:`:

```yaml
packages:
  living_room: !include
    file: common/room.yaml
    vars:
      room_id: living_room            # unique, letters/digits/underscore
      room_name: "Living Room"        # shown in Home Assistant
      patterns_file: patterns/living_room.yaml
```

Add, remove or rename blocks to get 1 to N fans. All of them share the single `remote_transmitter` (`rf_tx`);
ESPHome serializes transmissions that overlap, so two fans can be commanded at the same time without a queue.

### What shows up in Home Assistant
For every room (via ESPHome `devices:` / `areas:`):
* a **device** "Living Room Fan" assigned to the area "Living Room";
* one **fan** entity: OFF, 6 speeds and the "Breeze" preset;
* **18 buttons**: Fan Off, Speed 1-6, Breeze, Summer-Winter, Timer 1/2/4 h, Light, Warm/Neutral/Cold Light, Dimmer Up/Down.

The fan entity is **assumed-state** (the remote is one-way): it **never transmits at boot or when restoring state** and
follows whatever the buttons or actions send. Presses on the physical remote cannot be observed.
Delete the `fan:` or the `button:` block in `common/room.yaml` if you only want one of them.

## Hub
```yaml
remote_transmitter:
  id: rf_tx                    # to transmit; and/or receiver_id: to analyze (at least one)
  pin: GPIO5                   # GDO0 del CC1101
  carrier_duty_percent: 100%   # sin portadora de software: la genera el CC1101
  on_transmit:
    then:
      - cc1101.begin_tx: cc1101_radio
  on_complete:
    then:
      - cc1101.set_idle: cc1101_radio
```

### Actions
With several hubs, say which one with `id`:
```yaml
- arteconfort_433.send_pattern:
    id: radio_bedroom
    pattern: fanspeed3
- arteconfort_433.send_code:
    id: radio_office
    code: 0x3D41BAE4
    bits: 30        # optional
    repeat: 2       # optional
```

## Learning the codes and measuring the timing
Flash the capture firmware that matches your board (**only one**, do not mix it with another YAML):
`arteconfort_capture_rx.yaml` for the ESP32 or `arteconfort_capture_rx_esp8266.yaml` for the NodeMCU, with the same
wiring as the transmitter node, and press keys of the original remote next to the CC1101. Each file is a complete
firmware with its own `esp32:` or `esp8266:` block; copying parts into another YAML gives the error
*Found multiple target platform blocks*. The hub (with `receiver_id`) decodes each reception without knowing its
timing in advance:

```
Received 0x3D41BAE4 (30 bits) = pattern 'neutrallight'
  30 bits, then an idle gap of ~5000 us (the mark before it is the last bit)
  Short ~380 us, long ~1130 us
  YAML: bits: 30, gap_us: 5000, sync_mark_us: 0, sync_space_us: 0, short_us: 380, long_us: 1130
```
Copy the code to `patterns/*.yaml` and the last line to your transmitter's hub. A press arrives as several repetitions
glued together; the log says how many, and warns with `Reception of N pulses contains M distinct code(s)` if they differ.
Raw pulses are dumped too.

## Frame format (measured on a real remote)
Each frame is **30 bits back to back, then 5000 us of silence**, repeated unchanged (bit 1 = mark ~1130 / space ~380 us;
bit 0 = mark ~380 / space ~1130 us; MSB first). There is no sync field: what looks like a "sync" (short mark + ~6130 us
space) is the **30th bit with value 0** (mark 380 + space 1130) plus the 5000 us gap; with value 1 it reads mark 1130 +
space 380 + 5000 = 5380 us. A model of "fixed sync + 29 bits" therefore works for keys whose last bit is 0 and fails for
those whose last bit is 1 (in the tested remote: Cold Light and Timer 1 h). Defaults: `bits: 30`, `gap_us: 5000`,
`sync_mark_us: 0`, `sync_space_us: 0`, frequency `433.92MHz`.

## Protocols: `arteconfort`, `sulion`, `auto`
Each hub has a `protocol:` option that selects a preset (the individual options below override it):

|                                    | `arteconfort` (default) | `sulion`                           |
|------------------------------------|-------------------------|------------------------------------|
| Bits (MSB first)                   | 30                      | 32 (first 16 bits always `0xCA38`) |
| Bit 1 (mark / space)               | ~1130 / ~380 us         | ~1190 / ~430 us                    |
| Bit 0 (mark / space)               | ~380 / ~1130 us         | ~380 / ~1240 us                    |
| Trailer                            | none                    | short final mark (~380 us)         |
| Gap between repetitions (`gap_us`) | 5000 us                 | 12000 us (estimate, +-1 ms)        |

`protocol: auto` is only for receive-only hubs (the capture and sweep firmwares use it): it tries every known protocol and
prints which one matched (`Protocol: sulion (use protocol: sulion in the hub)`).

Per-hub options (also usable as `vars:` in `common/room.yaml`): `protocol`, `bits`, `gap_us`, `sync_mark_us`, `sync_space_us`,
`short_us`/`long_us`, `trailer_mark_us`, `bit1_mark_us`, `bit1_space_us`, `bit0_mark_us`, `bit0_space_us` (microseconds).

Using a Sulion remote on a room (one device can mix protocols, one per fan):

```yaml
  office: !include
    file: common/room.yaml
    vars:
      room_id: office
      room_name: "Office"
      patterns_file: patterns/sulion_example.yaml   # rename the keys to fanoff, fanspeed1, ... as you learn them
      protocol: sulion
```

## Troubleshooting
**Only noise (receptions of 2-5 pulses).** A real press is ~240 pulses. The remote most likely does not transmit on the
configured frequency (the 434.05 MHz of the original project was not right for the tested remote; the default is now
433.92 MHz). Flash **only one** sweep firmware: `arteconfort_scan_freq.yaml` (ESP32) or `arteconfort_scan_freq_esp8266.yaml`
(NodeMCU). It steps through 433.05-434.85 MHz (one step every 3 s, ~40 s per lap) and logs the frequency. Hold a key 1-2 cm
from the module. When `Received 0x...` (or receptions with many pulses) shows up, note the frequency and set it in
`cc1101: frequency:` of the **transmitter** node and of the capture firmware. If nothing appears in the whole band, check
that the module is 433 MHz (not 315/868), the antenna, the wiring (GDO0 -> GPIO4) and the remote's battery.

**Some keys do not work.** If always the same keys fail and the rest work, it is not power or frequency: what is sent for
those keys differs from what the remote sends. With the capture firmware, press that key **once** on the original remote
and compare it (repetitions, bits, distinct codes) with a key that works. With a very high `repeat`, toggle-type keys
(light, timer) may act several times: use the lowest value that works.

**Occasionally the fan ignores a press.** Raise `arteconfort_repeat` (8-12), keep `output_power` at 10-11 dBm, make sure the
antenna is connected, use a stable 3.3 V supply (100 uF capacitor near the module) and, on the ESP8266, try
`wifi: power_save_mode: none`.

## Tests
`tests/run_host_tests.sh` (needs `pip install esphome` and `g++`, ~1 min):
* **Compile matrix**: the sources build with `-Wall -Wextra` against the real ESPHome headers in the four possible
  cases (with/without buttons, with/without fan).
* **Encoder and analyzer**: bit-exact equivalence with the old v5.8 waveform, round trip, +-10 % jitter, receiver bias,
  truncated last frame, pure noise, other remotes, 8 and 64 bits, and the real 30-bit frames captured from a remote.
* **Behavior** on ESPHome's real core with a mock transmitter: what is sent, validation, the fan never transmitting at
  boot, following the buttons, and no double publishing.

`MUTATE=1` breaks the encoder on purpose (the tests must then fail) and `QUICK=1` skips the behavior part.

## Status
Verified: `esphome config` for all YAML files (ESP32, ESP8266, capture, sweep, three rooms), generated C++ for three hubs /
three fans / 54 buttons / three devices, the host tests above, and decoding of real captures from a remote.
**Not verified:** a full firmware build (PlatformIO could not download toolchains in the test environment), the minimum
ESPHome version that ships `cc1101`, and sustained operation with three real fans.

## License
[MIT](LICENSE)