import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.core import CORE
from esphome.components import time as time_
from esphome.const import CONF_ID, CONF_NUMBER, PLATFORM_ESP8266, PLATFORM_ESP32, KEY_CORE, KEY_TARGET_PLATFORM
from esphome.components.esp32 import include_builtin_idf_component

CODEOWNERS = ["@misterblack1","@pulleralarm"]
ESP_PLATFORMS = [PLATFORM_ESP8266, PLATFORM_ESP32]

aip33628_ns = cg.esphome_ns.namespace("aip33628")
Aip33628Panel = aip33628_ns.class_("Aip33628Panel", cg.Component)

CONF_CLK_PIN = "clk_pin"
CONF_DATA_PIN = "data_pin"
CONF_CLK2_PIN = "clk2_pin"
CONF_DATA2_PIN = "data2_pin"
CONF_MAX_CURRENT = "max_current"

def get_target_platform() -> str:
    return CORE.data[KEY_CORE][KEY_TARGET_PLATFORM]

def _fast_gpio_pin(value):
    """Validate a pin usable by the ESP8266 GPOS/GPOC fast GPIO registers.

    The display scan clocks both buses with direct GPIO register writes. Those
    registers cover GPIO0 through GPIO15; GPIO16 uses a separate register.
    GPIO6 through GPIO11 are connected to the module flash and cannot be used.
    """
    pin = value[CONF_NUMBER]
    if pin < 0 or pin > 15:
        raise cv.Invalid(
            f"GPIO{pin} cannot be driven by the ESP8266 GPOS/GPOC registers. "
            "Use a GPIO from 0 through 15."
        )
    if 6 <= pin <= 11:
        raise cv.Invalid(
            f"GPIO{pin} is reserved for the ESP8266 flash interface. "
            "Pick another GPIO."
        )
    return value

def _low_bank_pin(value):
    """Both buses are clocked from one write to the low GPIO output register.

    That register only reaches GPIO0 to GPIO31. A pin above that would need a
    second register and a second store per edge, which is most of the reason
    the frame send is fast enough to subdivide a COM slot at all.
    """
    pin = value[CONF_NUMBER]
    if pin >= 32:
        raise cv.Invalid(
            f"GPIO{pin} is above GPIO31, and the display scan drives all four "
            "lines from the low output register. Pick a pin below GPIO32."
        )
    return value

def _validate_pin(value):
    if get_target_platform() == PLATFORM_ESP32:
        return _low_bank_pin(value)
    else: # ESP8266
        return _fast_gpio_pin(value)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Aip33628Panel),
        cv.Required(CONF_CLK_PIN): cv.All(
            pins.internal_gpio_output_pin_schema, _validate_pin
        ),
        cv.Required(CONF_DATA_PIN): cv.All(
            pins.internal_gpio_output_pin_schema, _validate_pin
        ),
        cv.Optional(CONF_CLK2_PIN): cv.All(
            pins.internal_gpio_output_pin_schema, _validate_pin
        ),
        cv.Required(CONF_DATA2_PIN): cv.All(
            pins.internal_gpio_output_pin_schema, _validate_pin
        ),
        # A fixed ceiling on IS[3:0]. It must not depend on what is on screen,
        # or the whole panel changes brightness whenever the colon blinks.
        cv.Optional(CONF_MAX_CURRENT, default=15): cv.int_range(min=0, max=15),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    if get_target_platform() == PLATFORM_ESP32:
        include_builtin_idf_component("esp_driver_gptimer")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    clk = await cg.gpio_pin_expression(config[CONF_CLK_PIN])
    data = await cg.gpio_pin_expression(config[CONF_DATA_PIN])
    if CONF_CLK2_PIN in config:
        clk2 = await cg.gpio_pin_expression(config[CONF_CLK2_PIN])
    else:
        clk2 = cg.nullptr
    data2 = await cg.gpio_pin_expression(config[CONF_DATA2_PIN])
    cg.add(var.set_pins(clk, data, clk2, data2))

    cg.add(var.set_max_current(config[CONF_MAX_CURRENT]))
