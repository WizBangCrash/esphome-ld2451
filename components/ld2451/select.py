import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_LD2451_ID, LD2451Component, ld2451_ns

DEPENDENCIES = ["ld2451"]

LD2451DirectionSelect = ld2451_ns.class_(
    "LD2451DirectionSelect", select.Select, cg.Component
)

CONF_DETECTION_DIRECTION = "detection_direction"
OPTIONS = ["Away", "Towards", "All"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        cv.Optional(CONF_DETECTION_DIRECTION): select.select_schema(
            LD2451DirectionSelect,
            icon="mdi:swap-horizontal",
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    if direction_config := config.get(CONF_DETECTION_DIRECTION):
        s = await select.new_select(direction_config, options=OPTIONS)
        await cg.register_component(s, direction_config)
        cg.add(s.set_parent(hub))
