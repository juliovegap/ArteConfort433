"""ArteConfort 433 MHz ceiling-fan protocol for ESPHome (v6).

This component only *encodes/decodes* the protocol. The radio hardware is handled by
ESPHome's own components: cc1101 (SPI radio, OOK, async TX/RX) + remote_transmitter /
remote_receiver (precise waveform on GDO0). No third-party libraries, no Arduino-only code.

  transmitter_id: send patterns (buttons, fan, actions)
  receiver_id:    analyze frames from a real remote (learning / measuring the sync)
At least one of them is required.
"""

from esphome import automation, final_validate as fv
import esphome.codegen as cg
from esphome.components import remote_base
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = []
# remote_base is needed by the headers even when only a platform (button/fan) uses the hub.
AUTO_LOAD = ["remote_base"]
MULTI_CONF = True

CONF_TRANSMITTER_ID = remote_base.CONF_TRANSMITTER_ID
CONF_RECEIVER_ID = remote_base.CONF_RECEIVER_ID
CONF_BITS = "bits"
CONF_REPEAT = "repeat"
CONF_GAP_US = "gap_us"
CONF_SYNC_MARK_US = "sync_mark_us"
CONF_SYNC_SPACE_US = "sync_space_us"
CONF_SHORT_US = "short_us"
CONF_LONG_US = "long_us"
CONF_PROTOCOL = "protocol"
CONF_TRAILER_MARK_US = "trailer_mark_us"
CONF_BIT1_MARK_US = "bit1_mark_us"
CONF_BIT1_SPACE_US = "bit1_space_us"
CONF_BIT0_MARK_US = "bit0_mark_us"
CONF_BIT0_SPACE_US = "bit0_space_us"
CONF_PATTERNS = "patterns"
CONF_PATTERN = "pattern"
CONF_CODE = "code"
CONF_ARTECONFORT_ID = "arteconfort_id"

arteconfort_433_ns = cg.esphome_ns.namespace("arteconfort_433")
ArteConfort433 = arteconfort_433_ns.class_(
    "ArteConfort433", cg.Component, remote_base.RemoteReceiverListener
)
SendPatternAction = arteconfort_433_ns.class_(
    "SendPatternAction", automation.Action, cg.Parented.template(ArteConfort433)
)
SendCodeAction = arteconfort_433_ns.class_(
    "SendCodeAction", automation.Action, cg.Parented.template(ArteConfort433)
)

PATTERN_NAME = cv.All(cv.string_strict, cv.Length(min=1, max=48))


# Built-in protocol presets. Any option written explicitly in the hub overrides the preset value.
#   arteconfort: 30 bits, bit 1 = 1130/380 us, bit 0 = 380/1130 us (mark/space), 5 ms idle gap.
#   sulion:      32 bits, bit 1 = 1190/430 us, bit 0 = 380/1240 us, a final 380 us mark, ~12 ms idle gap.
#   auto:        receive-only; the analyzer tries every protocol above.
PRESETS = {
    "arteconfort": {
        CONF_BITS: 30,
        CONF_GAP_US: 5000,
        CONF_SYNC_MARK_US: 0,
        CONF_SYNC_SPACE_US: 0,
        CONF_SHORT_US: 380,
        CONF_LONG_US: 1130,
        CONF_TRAILER_MARK_US: 0,
    },
    "sulion": {
        CONF_BITS: 32,
        CONF_GAP_US: 12000,
        CONF_SYNC_MARK_US: 0,
        CONF_SYNC_SPACE_US: 0,
        CONF_SHORT_US: 380,
        CONF_LONG_US: 1240,
        CONF_BIT1_MARK_US: 1190,
        CONF_BIT1_SPACE_US: 430,
        CONF_BIT0_MARK_US: 380,
        CONF_BIT0_SPACE_US: 1240,
        CONF_TRAILER_MARK_US: 380,
    },
}
PRESETS["auto"] = PRESETS["arteconfort"]


def _value(config, key):
    """Explicit option if given, otherwise the value from the selected protocol preset (or None)."""
    if key in config:
        return config[key]
    return PRESETS[config[CONF_PROTOCOL]].get(key)


def _validate_protocol(config):
    if config[CONF_PROTOCOL] == "auto" and CONF_TRANSMITTER_ID in config:
        raise cv.Invalid(
            "protocol: auto only makes sense for receive-only hubs (the analyzer); "
            "a transmitting hub needs a concrete protocol",
            path=[CONF_PROTOCOL],
        )
    return config


def _validate_codes_fit(config):
    if config[CONF_PROTOCOL] == "auto":
        return config  # the patterns may belong to either protocol
    bits = _value(config, CONF_BITS)
    for name, code in config[CONF_PATTERNS].items():
        if code >> bits:
            raise cv.Invalid(
                f"Pattern '{name}' = 0x{code:X} does not fit in {bits} bits "
                f"(check '{CONF_BITS}' or the code)",
                path=[CONF_PATTERNS, name],
            )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ArteConfort433),
            cv.Optional(CONF_TRANSMITTER_ID): cv.use_id(
                remote_base.RemoteTransmitterBase
            ),
            cv.Optional(CONF_RECEIVER_ID): cv.use_id(remote_base.RemoteReceiverBase),
            cv.Optional(CONF_PROTOCOL, default="arteconfort"): cv.one_of(
                "arteconfort", "sulion", "auto", lower=True
            ),
            # The options below default to the values of the selected `protocol`.
            cv.Optional(CONF_BITS): cv.int_range(min=1, max=64),
            cv.Optional(CONF_REPEAT, default=4): cv.int_range(min=1, max=20),
            cv.Optional(CONF_GAP_US): cv.int_range(
                min=0, max=1000000
            ),
            cv.Optional(CONF_SYNC_MARK_US): cv.int_range(
                min=0, max=100000
            ),
            cv.Optional(CONF_SYNC_SPACE_US): cv.int_range(
                min=0, max=100000
            ),
            cv.Optional(CONF_SHORT_US): cv.int_range(min=50, max=20000),
            cv.Optional(CONF_LONG_US): cv.int_range(min=50, max=20000),
            cv.Optional(CONF_TRAILER_MARK_US): cv.int_range(min=0, max=20000),
            # Optional per-bit overrides for remotes whose short mark differs from their short space.
            cv.Optional(CONF_BIT1_MARK_US): cv.int_range(min=50, max=20000),
            cv.Optional(CONF_BIT1_SPACE_US): cv.int_range(min=50, max=20000),
            cv.Optional(CONF_BIT0_MARK_US): cv.int_range(min=50, max=20000),
            cv.Optional(CONF_BIT0_SPACE_US): cv.int_range(min=50, max=20000),
            cv.Optional(CONF_PATTERNS, default={}): cv.Schema(
                {PATTERN_NAME: cv.uint64_t}
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_TRANSMITTER_ID, CONF_RECEIVER_ID),
    _validate_protocol,
    _validate_codes_fit,
)


def require_pattern(hub_id, pattern, path):
    """Final-validation helper shared by the button and fan platforms:
    a typo in a pattern name becomes a config error instead of a runtime warning."""
    fconf = fv.full_config.get()
    try:
        hub_path = fconf.get_path_for_id(hub_id)[:-1]
        hub_conf = fconf.get_config_for_path(hub_path)
    except KeyError:
        return
    patterns = hub_conf.get(CONF_PATTERNS, {})
    if pattern not in patterns:
        available = ", ".join(sorted(patterns)) or "(none defined)"
        raise cv.Invalid(
            f"Pattern '{pattern}' is not defined in arteconfort_433 '{hub_id}'. "
            f"Available: {available}",
            path=path,
        )


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TRANSMITTER_ID in config:
        tx = await cg.get_variable(config[CONF_TRANSMITTER_ID])
        cg.add(var.set_transmitter(tx))
    if CONF_RECEIVER_ID in config:
        rx = await cg.get_variable(config[CONF_RECEIVER_ID])
        cg.add(rx.register_listener(var))
        cg.add(var.set_receiving(True))
    cg.add(var.set_protocol(config[CONF_PROTOCOL]))
    cg.add(var.set_auto_detect(config[CONF_PROTOCOL] == "auto"))
    cg.add(var.set_bits(_value(config, CONF_BITS)))
    cg.add(var.set_repeat(config[CONF_REPEAT]))
    cg.add(var.set_gap_us(_value(config, CONF_GAP_US)))
    cg.add(var.set_sync_mark_us(_value(config, CONF_SYNC_MARK_US)))
    cg.add(var.set_sync_space_us(_value(config, CONF_SYNC_SPACE_US)))
    cg.add(var.set_short_us(_value(config, CONF_SHORT_US)))
    cg.add(var.set_long_us(_value(config, CONF_LONG_US)))
    cg.add(var.set_trailer_mark_us(_value(config, CONF_TRAILER_MARK_US)))
    for key, setter in (
        (CONF_BIT1_MARK_US, var.set_one_mark_us),
        (CONF_BIT1_SPACE_US, var.set_one_space_us),
        (CONF_BIT0_MARK_US, var.set_zero_mark_us),
        (CONF_BIT0_SPACE_US, var.set_zero_space_us),
    ):
        value = _value(config, key)
        if value is not None:
            cg.add(setter(value))

    for name, code in config[CONF_PATTERNS].items():
        cg.add(var.add_pattern(name, code))


SEND_PATTERN_SCHEMA = cv.maybe_simple_value(
    {
        cv.GenerateID(): cv.use_id(ArteConfort433),
        cv.Required(CONF_PATTERN): cv.templatable(cv.string),
    },
    key=CONF_PATTERN,
)


@automation.register_action(
    "arteconfort_433.send_pattern",
    SendPatternAction,
    SEND_PATTERN_SCHEMA,
    synchronous=True,
)
async def send_pattern_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PATTERN], args, cg.std_string)
    cg.add(var.set_pattern(template_))
    return var


SEND_CODE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(ArteConfort433),
        cv.Required(CONF_CODE): cv.templatable(cv.uint64_t),
        cv.Optional(CONF_BITS): cv.templatable(cv.int_range(min=1, max=64)),
        cv.Optional(CONF_REPEAT): cv.templatable(cv.int_range(min=1, max=20)),
    }
)


@automation.register_action(
    "arteconfort_433.send_code",
    SendCodeAction,
    SEND_CODE_SCHEMA,
    synchronous=True,
)
async def send_code_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_code(await cg.templatable(config[CONF_CODE], args, cg.uint64)))
    if CONF_BITS in config:
        cg.add(var.set_bits(await cg.templatable(config[CONF_BITS], args, cg.uint8)))
    if CONF_REPEAT in config:
        cg.add(
            var.set_repeat(await cg.templatable(config[CONF_REPEAT], args, cg.uint8))
        )
    return var
