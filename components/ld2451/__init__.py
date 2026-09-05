import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@WizBangCrash"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor", "number", "select", "switch", "button"]
MULTI_CONF = True

ld2451_ns = cg.esphome_ns.namespace("ld2451")
LD2451Component = ld2451_ns.class_("LD2451Component", cg.Component, uart.UARTDevice)

LD2451Direction = ld2451_ns.enum("LD2451Direction", is_class=True)
DIRECTION_OPTIONS = {
    "Away": LD2451Direction.AWAY,
    "Toward": LD2451Direction.TOWARD,
    "All": LD2451Direction.ALL,
}

CONF_LD2451_ID = "ld2451_id"

# Actions, usable from automations / HA services:
#   ld2451.factory_reset:  id: my_ld2451
#   ld2451.restart:        id: my_ld2451
#   ld2451.refresh_config: id: my_ld2451
FactoryResetAction = ld2451_ns.class_("LD2451FactoryResetAction", automation.Action)
RestartAction = ld2451_ns.class_("LD2451RestartAction", automation.Action)
RefreshConfigAction = ld2451_ns.class_("LD2451RefreshConfigAction", automation.Action)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2451Component),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2451",
    baud_rate=256000,
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


LD2451_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(LD2451Component),
    }
)


@automation.register_action(
    "ld2451.factory_reset", FactoryResetAction, LD2451_ACTION_SCHEMA, synchronous=True
)
async def ld2451_factory_reset_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "ld2451.restart", RestartAction, LD2451_ACTION_SCHEMA, synchronous=True
)
async def ld2451_restart_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "ld2451.refresh_config", RefreshConfigAction, LD2451_ACTION_SCHEMA, synchronous=True
)
async def ld2451_refresh_config_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
