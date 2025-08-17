import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
)

DEPENDENCIES = ["uart"]

rs200_ns = cg.esphome_ns.namespace("rs200")
RS200Component = rs200_ns.class_("RS200Component", cg.PollingComponent, uart.UARTDevice)

CONF_RAINFALL = "rainfall_status"
CONF_SYSTEM = "system_status"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(RS200Component),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_RAINFALL): sensor.sensor_schema(
        unit_of_measurement="",
        icon="mdi:water",
        accuracy_decimals=0,
    ),
    cv.Required(CONF_SYSTEM): sensor.sensor_schema(
        unit_of_measurement="",
        icon="mdi:alert",
        accuracy_decimals=0,
    ),
}).extend(cv.polling_component_schema("5s"))

async def to_code(config):
    uart_var = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_var)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    rain_sensor = await sensor.new_sensor(config[CONF_RAINFALL])
    system_sensor = await sensor.new_sensor(config[CONF_SYSTEM])

    cg.add(var.set_rainfall_sensor(rain_sensor))
    cg.add(var.set_system_sensor(system_sensor))
