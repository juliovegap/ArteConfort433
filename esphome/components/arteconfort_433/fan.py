import esphome.codegen as cg
from esphome.components import fan
import esphome.config_validation as cv

from . import (
    CONF_ARTECONFORT_ID,
    PATTERN_NAME,
    ArteConfort433,
    arteconfort_433_ns,
    require_pattern,
)

DEPENDENCIES = ["arteconfort_433"]

CONF_OFF_PATTERN = "off_pattern"
CONF_SPEED_PATTERNS = "speed_patterns"
CONF_BREEZE_PATTERN = "breeze_pattern"

ArteConfort433Fan = arteconfort_433_ns.class_(
    "ArteConfort433Fan", cg.Component, fan.Fan
)

DEFAULT_SPEED_PATTERNS = [f"fanspeed{i}" for i in range(1, 7)]

CONFIG_SCHEMA = (
    fan.fan_schema(ArteConfort433Fan, default_restore_mode="RESTORE_DEFAULT_OFF")
    .extend(
        {
            cv.GenerateID(CONF_ARTECONFORT_ID): cv.use_id(ArteConfort433),
            cv.Optional(CONF_OFF_PATTERN, default="fanoff"): PATTERN_NAME,
            cv.Optional(
                CONF_SPEED_PATTERNS, default=DEFAULT_SPEED_PATTERNS
            ): cv.All(cv.ensure_list(PATTERN_NAME), cv.Length(min=1, max=20)),
            cv.Optional(CONF_BREEZE_PATTERN): PATTERN_NAME,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


def _final_validate(config):
    hub = config[CONF_ARTECONFORT_ID]
    require_pattern(hub, config[CONF_OFF_PATTERN], [CONF_OFF_PATTERN])
    for i, name in enumerate(config[CONF_SPEED_PATTERNS]):
        require_pattern(hub, name, [CONF_SPEED_PATTERNS, i])
    if CONF_BREEZE_PATTERN in config:
        require_pattern(hub, config[CONF_BREEZE_PATTERN], [CONF_BREEZE_PATTERN])
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = await fan.new_fan(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_ARTECONFORT_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_off_pattern(config[CONF_OFF_PATTERN]))
    for name in config[CONF_SPEED_PATTERNS]:
        cg.add(var.add_speed_pattern(name))
    if CONF_BREEZE_PATTERN in config:
        cg.add(var.set_breeze_pattern(config[CONF_BREEZE_PATTERN]))
