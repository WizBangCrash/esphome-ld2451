import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_SPEED,
    ICON_MOTION_SENSOR,
    STATE_CLASS_MEASUREMENT,
    UNIT_DEGREES,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_METER,
)

from . import CONF_LD2451_ID, LD2451Component, ld2451_ns

DEPENDENCIES = ["ld2451"]

CONF_TARGET_COUNT = "target_count"
CONF_TARGETS = "targets"
CONF_ANGLE = "angle"
CONF_DISTANCE = "distance"
CONF_SPEED = "speed"

ICON_DISTANCE = "mdi:map-marker-distance"
ICON_SPEED = "mdi:speedometer-slow"
ICON_TARGET_COUNT = "mdi:car-multiple"
ICON_ANGLE = "mdi:format-text-rotation-angle-up"

MAX_TARGETS = 5

TARGET_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon=ICON_ANGLE,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            filters=[
                {"throttle_with_priority": cv.TimePeriod(milliseconds=1000)},
            ],
        ),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            icon=ICON_DISTANCE,
            device_class=DEVICE_CLASS_DISTANCE,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            filters=[
                {"throttle_with_priority": cv.TimePeriod(milliseconds=1000)},
            ],
        ),
        cv.Optional(CONF_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            icon=ICON_SPEED,
            device_class=DEVICE_CLASS_SPEED,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            filters=[
                {"throttle_with_priority": cv.TimePeriod(milliseconds=1000)},
            ],
        ),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        cv.Optional(CONF_TARGET_COUNT): sensor.sensor_schema(
            icon=ICON_TARGET_COUNT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            filters=[
                {"throttle_with_priority": cv.TimePeriod(milliseconds=1000)},
            ],
        ),
        cv.Optional(CONF_TARGETS): cv.All(
            cv.ensure_list(TARGET_SCHEMA), cv.Length(max=MAX_TARGETS)
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    if target_count_config := config.get(CONF_TARGET_COUNT):
        sens = await sensor.new_sensor(target_count_config)
        cg.add(hub.set_target_count_sensor(sens))

    for i, target_config in enumerate(config.get(CONF_TARGETS, [])):
        if angle_config := target_config.get(CONF_ANGLE):
            sens = await sensor.new_sensor(angle_config)
            cg.add(hub.set_target_angle_sensor(i, sens))
        if distance_config := target_config.get(CONF_DISTANCE):
            sens = await sensor.new_sensor(distance_config)
            cg.add(hub.set_target_distance_sensor(i, sens))
        if speed_config := target_config.get(CONF_SPEED):
            sens = await sensor.new_sensor(speed_config)
            cg.add(hub.set_target_speed_sensor(i, sens))
