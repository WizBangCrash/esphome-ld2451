import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC, ICON_CHIP

from . import CONF_LD2451_ID, LD2451Component

DEPENDENCIES = ["ld2451"]

CONF_FIRMWARE_VERSION = "firmware_version"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
            icon=ICON_CHIP,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    var = await cg.get_variable(config[CONF_LD2451_ID])

    if fw_config := config.get(CONF_FIRMWARE_VERSION):
        s = await text_sensor.new_text_sensor(fw_config)
        cg.add(var.set_firmware_version_text_sensor(s))
