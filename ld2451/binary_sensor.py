import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_MOVING, DEVICE_CLASS_OCCUPANCY

from . import CONF_LD2451_ID, LD2451Component

DEPENDENCIES = ["ld2451"]

CONF_HAS_TARGET = "has_target"
CONF_HAS_APPROACHING_TARGET = "has_approaching_target"
CONF_TARGETS = "targets"
CONF_APPROACHING = "approaching"

ICON_HAS_TARGET = "mdi:shield-car"
ICON_APPROACHING_TARGET = "mdi:car-traction-control"

MAX_TARGETS = 5

TARGET_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_APPROACHING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOVING,
            icon=ICON_APPROACHING_TARGET,
            filters=[{"settle": cv.TimePeriod(milliseconds=250)}],
            ),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        cv.Optional(CONF_HAS_TARGET): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_OCCUPANCY,
            icon=ICON_HAS_TARGET,
            filters=[{"settle": cv.TimePeriod(milliseconds=250)}],
        ),
        cv.Optional(CONF_HAS_APPROACHING_TARGET): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOVING,
            icon=ICON_APPROACHING_TARGET,
            filters=[{"settle": cv.TimePeriod(milliseconds=250)}],
        ),
        cv.Optional(CONF_TARGETS): cv.All(
            cv.ensure_list(TARGET_SCHEMA), cv.Length(max=MAX_TARGETS)
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    if has_target_config := config.get(CONF_HAS_TARGET):
        sens = await binary_sensor.new_binary_sensor(has_target_config)
        cg.add(hub.set_has_target_binary_sensor(sens))

    if has_approaching_config := config.get(CONF_HAS_APPROACHING_TARGET):
        sens = await binary_sensor.new_binary_sensor(has_approaching_config)
        cg.add(hub.set_has_approaching_target_binary_sensor(sens))

    for i, target_config in enumerate(config.get(CONF_TARGETS, [])):
        if approaching_config := target_config.get(CONF_APPROACHING):
            sens = await binary_sensor.new_binary_sensor(approaching_config)
            cg.add(hub.set_target_approaching_binary_sensor(i, sens))
