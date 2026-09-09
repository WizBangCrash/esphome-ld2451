import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    ENTITY_CATEGORY_CONFIG,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_METER,
    UNIT_SECOND,
)

from . import CONF_LD2451_ID, LD2451Component, ld2451_ns

DEPENDENCIES = ["ld2451"]

LD2451Number = ld2451_ns.class_("LD2451Number", number.Number, cg.Component)
LD2451NumberField = ld2451_ns.enum("LD2451NumberField", is_class=True)

CONF_MAX_DISTANCE = "max_distance"
CONF_MIN_SPEED = "min_speed"
CONF_NO_TARGET_DELAY = "no_target_delay"
CONF_SNR_THRESHOLD = "snr_threshold"
CONF_DEFAULT_VALUE = "default_value"

# (yaml key, unit, min, max, step, icon, field enum member)
NUMBERS = {
    CONF_MAX_DISTANCE: (UNIT_METER, 10, 100, 1, "mdi:arrow-expand-horizontal", "MAX_DISTANCE"),
    CONF_MIN_SPEED: (UNIT_KILOMETER_PER_HOUR, 0, 120, 1, "mdi:speedometer-slow", "MIN_SPEED"),
    CONF_NO_TARGET_DELAY: (UNIT_SECOND, 0, 30, 1, "mdi:timer-outline", "NO_TARGET_DELAY"),
    CONF_SNR_THRESHOLD: ("", 3, 8, 1, "mdi:signal", "SNR_THRESHOLD"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        **{
            cv.Optional(key): number.number_schema(
                LD2451Number,
                unit_of_measurement=unit,
                icon=icon,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ).extend(
                {
                    cv.Optional(CONF_DEFAULT_VALUE): cv.int_range(
                        min=_min, max=_max
                    ),
                }
            )
            for key, (unit, _min, _max, _step, icon, _field) in NUMBERS.items()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    for key, (_unit, min_val, max_val, step, _icon, field) in NUMBERS.items():
        if key not in config:
            continue
        n = await number.new_number(
            config[key], min_value=min_val, max_value=max_val, step=step
        )
        await cg.register_component(n, config[key])
        cg.add(n.set_parent(hub))
        cg.add(n.set_field(getattr(LD2451NumberField, field)))
        if CONF_DEFAULT_VALUE in config[key]:
            cg.add(n.set_default_value(config[key][CONF_DEFAULT_VALUE]))
