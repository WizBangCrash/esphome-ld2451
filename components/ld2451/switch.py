import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

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

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        cv.Optional(CONF_REQUIRE_MULTIPLE_DETECTIONS): switch.switch_schema(
            LD2451MultiTriggerSwitch,
            icon="mdi:filter-check-outline",
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_BLUETOOTH): switch.switch_schema(
            LD2451BluetoothSwitch,
            icon="mdi:bluetooth",
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    if sw_config := config.get(CONF_REQUIRE_MULTIPLE_DETECTIONS):
        s = await switch.new_switch(sw_config)
        await cg.register_component(s, sw_config)
        cg.add(s.set_parent(hub))

    if bt_config := config.get(CONF_BLUETOOTH):
        s = await switch.new_switch(bt_config)
        await cg.register_component(s, bt_config)
        cg.add(s.set_parent(hub))
