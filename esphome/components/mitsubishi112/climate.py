import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, remote_transmitter, sensor
from esphome.const import CONF_ID, CONF_SENSOR

AUTO_LOAD = ['sensor']

mitsubishi112_ns = cg.esphome_ns.namespace('mitsubishi112')
Mitsubishi112Climate = mitsubishi112_ns.class_('Mitsubishi112Climate',
                                               climate.Climate, cg.Component)

CONF_TRANSMITTER_ID = 'transmitter_id'
CONF_SUPPORTS_HEAT = 'supports_heat'
CONF_SUPPORTS_COOL = 'supports_cool'
CONF_SUPPORTS_AUTO = 'supports_auto'

CONFIG_SCHEMA = cv.All(climate.CLIMATE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(Mitsubishi112Climate),
    cv.GenerateID(CONF_TRANSMITTER_ID): cv.use_id(remote_transmitter.RemoteTransmitterComponent),
    cv.Optional(CONF_SUPPORTS_COOL, default=True): cv.boolean,
    cv.Optional(CONF_SUPPORTS_HEAT, default=True): cv.boolean,
    cv.Optional(CONF_SUPPORTS_AUTO, default=True): cv.boolean,
    cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
}).extend(cv.COMPONENT_SCHEMA))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield climate.register_climate(var, config)

    cg.add(var.set_supports_cool(config[CONF_SUPPORTS_COOL]))
    cg.add(var.set_supports_heat(config[CONF_SUPPORTS_HEAT]))
    cg.add(var.set_supports_heat(config[CONF_SUPPORTS_AUTO]))
    if CONF_SENSOR in config:
        sens = yield cg.get_variable(config[CONF_SENSOR])
        cg.add(var.set_sensor(sens))

    transmitter = yield cg.get_variable(config[CONF_TRANSMITTER_ID])
    cg.add(var.set_transmitter(transmitter))
