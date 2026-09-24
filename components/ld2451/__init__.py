import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import CONF_ID
from esphome.core import ID
from esphome.cpp_generator import MockObj, TemplateArgsType
from esphome.types import ConfigType

CODEOWNERS = ["@WizBangCrash"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor", "number", "select", "switch", "button"]
MULTI_CONF = True

ld2451_ns = cg.esphome_ns.namespace("ld2451")
LD2451Component = ld2451_ns.class_("LD2451Component", cg.Component, uart.UARTDevice)

CONF_LD2451_ID = "ld2451_id"
CONF_ON_CONFIG_UPDATE = "on_config_update"
# Must match LD2451_MAX_TARGETS in ld2451.h
MAX_TARGETS = 5

# Actions, usable from automations / HA services:
#   ld2451.factory_reset:  id: my_ld2451
#   ld2451.restart:        id: my_ld2451
#   ld2451.refresh_config: id: my_ld2451
FactoryResetAction = ld2451_ns.class_("LD2451FactoryResetAction", automation.Action)
RestartAction = ld2451_ns.class_("LD2451RestartAction", automation.Action)
RefreshConfigAction = ld2451_ns.class_("LD2451RefreshConfigAction", automation.Action)

CONFIG_SCHEMA = (cv.Schema({
    cv.GenerateID(): cv.declare_id(LD2451Component),
    cv.Optional(CONF_ON_CONFIG_UPDATE): automation.validate_automation({}),
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

# Fires once the radar's target detection and sensitivity config has settled:
# every requested change has been written and read back (or given up on), and
# no config reads or writes remain queued.
_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(CONF_ON_CONFIG_UPDATE, "add_on_config_update_callback"),
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2451",
    baud_rate=256000,
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)


LD2451_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(LD2451Component),
    }
)


@automation.register_action(
    "ld2451.factory_reset", FactoryResetAction, LD2451_ACTION_SCHEMA, synchronous=True
)
async def ld2451_factory_reset_to_code(
    config: ConfigType, action_id: ID, template_arg: cg.TemplateArguments, args: TemplateArgsType
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "ld2451.restart", RestartAction, LD2451_ACTION_SCHEMA, synchronous=True
)
async def ld2451_restart_to_code(
    config: ConfigType, action_id: ID, template_arg: cg.TemplateArguments, args: TemplateArgsType
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "ld2451.refresh_config", RefreshConfigAction, LD2451_ACTION_SCHEMA, synchronous=True
)
async def ld2451_refresh_config_to_code(
    config: ConfigType, action_id: ID, template_arg: cg.TemplateArguments, args: TemplateArgsType
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
