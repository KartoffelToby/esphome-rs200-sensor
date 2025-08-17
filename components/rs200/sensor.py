import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID, UNIT_EMPTY, ICON_WATER, DEVICE_CLASS_EMPTY

DEPENDENCIES = ["uart"]

rs200_ns = cg.esphome_ns.namespace("rs200")
RS200Sensor = rs200_ns.class_("RS200Sensor", cg.PollingComponent, uart.UARTDevice, sensor.Sensor)

CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_EMPTY,
    icon=ICON_WATER,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_EMPTY,
).extend({
    cv.GenerateID(): cv.declare_id(RS200Sensor),
}).extend(cv.polling_component_schema("5s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], uart.UARTComponent.PARENT)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await sensor.register_sensor(var, config)
