import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_CONFIG,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_METER,
    UNIT_SECOND,
    ICON_TIMER,
)
from esphome.types import ConfigType

from . import CONF_LD2451_ID, LD2451Component, ld2451_ns

DEPENDENCIES = ["ld2451"]

LD2451Number = ld2451_ns.class_("LD2451Number", number.Number, cg.Component)
LD2451NumberField = ld2451_ns.enum("LD2451NumberField", is_class=True)

CONF_MAX_DISTANCE = "max_distance"
CONF_MIN_SPEED = "min_speed"
CONF_NO_TARGET_DELAY = "no_target_delay"
CONF_SNR_THRESHOLD = "snr_threshold"
CONF_TRIGGER_COUNT = "trigger_count"

ICON_ARROW_EXPAND_HORIZONTAL = "mdi:arrow-expand-horizontal"
ICON_SPEED_SLOW = "mdi:speedometer-slow"

# (yaml key, unit, min, max, step, icon, field enum member, hub setter for state sync)
NUMBERS = {
    CONF_MAX_DISTANCE: (
        UNIT_METER,
        10,
        100,
        1,
        ICON_ARROW_EXPAND_HORIZONTAL,
        "LD2451_NUMBER_FIELD_MAX_DISTANCE",
        "set_max_distance_number",
    ),
    CONF_MIN_SPEED: (
        UNIT_KILOMETER_PER_HOUR,
        0,
        120,
        1,
        ICON_SPEED_SLOW,
        "LD2451_NUMBER_FIELD_MIN_SPEED",
        "set_min_speed_number",
    ),
    CONF_NO_TARGET_DELAY: (
        UNIT_SECOND,
        0,
        30,
        1,
        ICON_TIMER,
        "LD2451_NUMBER_FIELD_NO_TARGET_DELAY",
        "set_no_target_delay_number",
    ),
    CONF_SNR_THRESHOLD: ("", 3, 8, 1, "mdi:signal", "LD2451_NUMBER_FIELD_SNR_THRESHOLD", "set_snr_threshold_number"),
    CONF_TRIGGER_COUNT: ("", 0, 10, 1, "mdi:counter", "LD2451_NUMBER_FIELD_TRIGGER_COUNT", "set_trigger_count_number"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        **{
            cv.Optional(key): number.number_schema(
                LD2451Number,
                unit_of_measurement=unit,
                icon=icon,
                entity_category=ENTITY_CATEGORY_CONFIG,
            )
            for key, (unit, _min, _max, _step, icon, _field, _setter) in NUMBERS.items()
        },
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    for key, (_unit, min_val, max_val, step, _icon, field, setter) in NUMBERS.items():
        if key not in config:
            continue
        n = await number.new_number(
            config[key], min_value=min_val, max_value=max_val, step=step
        )
        await cg.register_component(n, config[key])
        cg.add(n.set_parent(hub))
        cg.add(n.set_field(getattr(LD2451NumberField, field)))
        cg.add(getattr(hub, setter)(n))
