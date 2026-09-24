import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG, DEVICE_CLASS_RESTART
from esphome.types import ConfigType

from . import CONF_LD2451_ID, LD2451Component, ld2451_ns

DEPENDENCIES = ["ld2451"]

LD2451Button = ld2451_ns.class_("LD2451Button", button.Button, cg.Component)
LD2451ButtonAction = ld2451_ns.enum("LD2451ButtonAction", is_class=True)

CONF_FACTORY_RESET = "factory_reset"
CONF_RESTART = "restart"
CONF_REFRESH_CONFIG = "refresh_config"

BUTTONS = {
    CONF_FACTORY_RESET: ("mdi:restore", None, "LD2451_BUTTON_ACTION_FACTORY_RESET"),
    CONF_RESTART: ("mdi:restart", DEVICE_CLASS_RESTART, "LD2451_BUTTON_ACTION_RESTART"),
    CONF_REFRESH_CONFIG: ("mdi:refresh", None, "LD2451_BUTTON_ACTION_REFRESH_CONFIG"),
}

def _button_schema(icon: str, device_class: str | None) -> cv.Schema:
    kwargs = {"icon": icon, "entity_category": ENTITY_CATEGORY_CONFIG}
    if device_class is not None:
        kwargs["device_class"] = device_class
    return button.button_schema(LD2451Button, **kwargs)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_LD2451_ID): cv.use_id(LD2451Component),
        **{
            cv.Optional(key): _button_schema(icon, device_class)
            for key, (icon, device_class, _action) in BUTTONS.items()
        },
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_LD2451_ID])

    for key, (_icon, _device_class, action) in BUTTONS.items():
        if key not in config:
            continue
        b = await button.new_button(config[key])
        await cg.register_component(b, config[key])
        cg.add(b.set_parent(hub))
        cg.add(b.set_action(getattr(LD2451ButtonAction, action)))
