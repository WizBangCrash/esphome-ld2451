import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_LD2451_ID, LD2451Component

DEPENDENCIES = ["ld2451"]

CONF_FIRMWARE_VERSION = "firmware_version"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
            icon="mdi:chip",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    if fw_config := config.get(CONF_FIRMWARE_VERSION):
        s = await text_sensor.new_text_sensor(fw_config)
        cg.add(hub.set_firmware_version_text_sensor(s))
