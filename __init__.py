# --- ESPHOME PYTHON BINDINGS ---
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome import pins
from esphome.const import CONF_ID, CONF_UART_ID


DEPENDENCIES = ["uart"]

# --- Custom configuration keys ---
CONF_HANDLER_ID = "handler"

# --- Define C++ classes ---
olimpia_bridge_ns = cg.esphome_ns.namespace("olimpia_bridge")
OlimpiaBridge = olimpia_bridge_ns.class_("OlimpiaBridge", cg.Component, uart.UARTDevice)
ModbusAsciiHandler = olimpia_bridge_ns.class_("ModbusAsciiHandler")

# --- Configuration schema ---
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(OlimpiaBridge),
            cv.GenerateID(CONF_HANDLER_ID): cv.declare_id(ModbusAsciiHandler),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required("re_pin"): pins.gpio_output_pin_schema,
            cv.Required("de_pin"): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

# --- Code generation ---
async def to_code(config):
    controller = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(controller, config)

    # Get the UART component variable and set the UART for the handler
    uart_var = await cg.get_variable(config[CONF_UART_ID])
    cg.add(controller.set_uart_parent(uart_var))

    # Setup RE/DE pins
    re_pin = await cg.gpio_pin_expression(config["re_pin"])
    de_pin = await cg.gpio_pin_expression(config["de_pin"])
    cg.add(controller.set_re_pin(re_pin))
    cg.add(controller.set_de_pin(de_pin))

    # Create and bind ModbusAsciiHandler
    handler = cg.new_Pvariable(config[CONF_HANDLER_ID])
    cg.add(controller.set_handler(handler))