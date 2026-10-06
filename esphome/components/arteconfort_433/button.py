import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import CONF_ID

from . import (
    CONF_ARTECONFORT_ID,
    CONF_PATTERN,
    ArteConfort433,
    arteconfort_433_ns,
    require_pattern,
)

DEPENDENCIES = ["arteconfort_433"]

ArteConfort433Button = arteconfort_433_ns.class_(
    "ArteConfort433Button", button.Button
)

CONFIG_SCHEMA = button.button_schema(ArteConfort433Button).extend(
    {
        cv.GenerateID(CONF_ARTECONFORT_ID): cv.use_id(ArteConfort433),
        cv.Required(CONF_PATTERN): cv.string_strict,
    }
)


def _final_validate(config):
    require_pattern(
        config[CONF_ARTECONFORT_ID], config[CONF_PATTERN], [CONF_PATTERN]
    )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await button.register_button(var, config)

    parent = await cg.get_variable(config[CONF_ARTECONFORT_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_pattern(config[CONF_PATTERN]))
