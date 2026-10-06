#!/usr/bin/env bash
# Host tests + compile checks against the real ESPHome headers.
# Needs: python3, g++ (C++20), `pip install esphome`.
# Optional: MUTATE=1 breaks the encoder on purpose; the tests must then FAIL.
#           QUICK=1 skips the behavior tests (they compile ESPHome's core, ~1-2 min).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

PK="$(python3 -c 'import esphome,os;print(os.path.dirname(esphome.__file__))')/components"

# build_tree NAME "button?" "fan?" : ESPHome generates its host-platform source tree for a
# configuration that contains exactly those components (remote_base always pulls binary_sensor),
# i.e. the same set of headers a real firmware build would have.
build_tree() {
  local name="$1" button="$2" fan="$3"
  mkdir -p "$name" && cat > "$name/host.yaml" <<YAML
esphome:
  name: hosttest
host:
logger:
binary_sensor:
  - platform: template
    name: bs
YAML
  if [ "$button" = 1 ]; then printf 'button:\n  - platform: template\n    name: b\n' >> "$name/host.yaml"; fi
  if [ "$fan" = 1 ];    then printf 'fan:\n  - platform: template\n    name: f\n'    >> "$name/host.yaml"; fi
  (cd "$name" && esphome compile --only-generate host.yaml >/dev/null 2>&1)
  mkdir -p "$name/overlay/esphome/components"
  ln -s "$PK/remote_base" "$name/overlay/esphome/components/remote_base"
  if [ "${MUTATE:-0}" = "1" ]; then
    cp -r "$HERE/../esphome/components/arteconfort_433" "$name/overlay/esphome/components/arteconfort_433"
    sed -i 's|push(false, bit ? t.short_us : t.long_us);|push(false, bit ? t.long_us : t.short_us);|' \
        "$name/overlay/esphome/components/arteconfort_433/arteconfort_433.h"
  else
    ln -s "$HERE/../esphome/components/arteconfort_433" "$name/overlay/esphome/components/arteconfort_433"
  fi
}

check_tree() {
  local name="$1"
  local SRC="$WORK/$name/.esphome/build/hosttest/src"
  local CXX="g++ -std=gnu++20 -Wall -Wextra -I$SRC -I$WORK/$name/overlay -DUSE_HOST"
  local COMP="$WORK/$name/overlay/esphome/components/arteconfort_433"
  $CXX -fsyntax-only "$COMP/arteconfort_433.cpp"
  $CXX -fsyntax-only "$COMP/arteconfort_fan.cpp"
  $CXX -fsyntax-only "$HERE/compile_check.cpp"
}

echo "== compile matrix (the component must build with or without button / fan)"
for cfg in "buttons+fan 1 1" "only-fan 0 1" "only-buttons 1 0" "neither 0 0"; do
  set -- $cfg
  build_tree "$1" "$2" "$3"
  check_tree "$1"
  echo "   ok: $1"
done

echo "== encoder + analyzer tests"
SRC="$WORK/buttons+fan/.esphome/build/hosttest/src"
g++ -std=gnu++20 -Wall -Wextra -I"$SRC" -I"$WORK/buttons+fan/overlay" -DUSE_HOST -o enc_test "$HERE/test_encoder.cpp"
./enc_test

if [ "${QUICK:-0}" = "1" ]; then exit 0; fi

echo "== behavior tests (real ESPHome core + mock transmitter)"
T="$WORK/buttons+fan"
SRC="$T/.esphome/build/hosttest/src"
export FLAGS="-std=gnu++20 -DUSE_HOST -DESPHOME_LOG_LEVEL=ESPHOME_LOG_LEVEL_DEBUG -fno-exceptions -Wno-sign-compare -w -I$SRC -I$T/overlay"
mkdir -p "$T/obj"
export OBJ="$T/obj"
{ ls "$SRC"/esphome/core/*.cpp "$SRC"/esphome/core/wake/*.cpp
  for c in host logger fan button binary_sensor; do ls "$SRC"/esphome/components/$c/*.cpp; done
  echo "$PK/remote_base/remote_base.cpp"
} | grep -v 'host/core.cpp' | xargs -P "$(nproc)" -I{} sh -c 'g++ $FLAGS -c "{}" -o "$OBJ/$(echo "{}" | md5sum | cut -c1-10).o"'
# host/core.cpp defines its own main(): rename it, the test brings its own.
g++ $FLAGS -Dmain=esphome_host_main_unused -c "$SRC/esphome/components/host/core.cpp" -o "$OBJ/hostcore.o"
COMP="$T/overlay/esphome/components/arteconfort_433"
g++ $FLAGS -c "$COMP/arteconfort_433.cpp" -o "$OBJ/hub.o"
g++ $FLAGS -c "$COMP/arteconfort_fan.cpp" -o "$OBJ/fan_c.o"
g++ $FLAGS -c "$HERE/test_behavior.cpp" -o "$OBJ/test.o"
g++ "$OBJ"/*.o -o behavior_test
./behavior_test
