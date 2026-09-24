import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG, ICON_BLUETOOTH

from esphome.types import ConfigType

from . import CONF_LD2451_ID, LD2451Component, ld2451_ns

DEPENDENCIES = ["ld2451"]

LD2451MultiTriggerSwitch = ld2451_ns.class_(
    "LD2451MultiTriggerSwitch", switch.Switch, cg.Component
)
LD2451BluetoothSwitch = ld2451_ns.class_(
    "LD2451BluetoothSwitch", switch.Switch, cg.Component
)

CONF_REQUIRE_MULTIPLE_DETECTIONS = "require_multiple_detections"
CONF_BLUETOOTH = "bluetooth"

ICON_FILTER_CHECK_OUTLINE = "mdi:filter-check-outline"

_LOGGER = logging.getLogger(__name__)


def _warn_multi_trigger_deprecated(config: ConfigType) -> ConfigType:
    if CONF_REQUIRE_MULTIPLE_DETECTIONS in config:
        _LOGGER.warning(
            "ld2451: 'require_multiple_detections' switch is deprecated and will be removed "
            "in a future release. Use the 'trigger_count' number (0-10) instead."
        )
    return config


CONFIG_SCHEMA = cv.All(cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        cv.Optional(CONF_REQUIRE_MULTIPLE_DETECTIONS): switch.switch_schema(
            LD2451MultiTriggerSwitch,
            icon=ICON_FILTER_CHECK_OUTLINE,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_BLUETOOTH): switch.switch_schema(
            LD2451BluetoothSwitch,
            icon=ICON_BLUETOOTH,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
    }
), _warn_multi_trigger_deprecated)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    if sw_config := config.get(CONF_REQUIRE_MULTIPLE_DETECTIONS):
        s = await switch.new_switch(sw_config)
        await cg.register_component(s, sw_config)
        cg.add(s.set_parent(hub))
        cg.add(hub.set_multi_trigger_switch(s))

    if bt_config := config.get(CONF_BLUETOOTH):
        s = await switch.new_switch(bt_config)
        await cg.register_component(s, bt_config)
        cg.add(s.set_parent(hub))
