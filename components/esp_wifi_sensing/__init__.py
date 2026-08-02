import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_GAIN_COMPENSATION = "gain_compensation"

DEPENDENCIES = ["wifi"]

esp_wifi_sensing_ns = cg.esphome_ns.namespace("esp_wifi_sensing")

ESPWiFiSensing = esp_wifi_sensing_ns.class_(
    "ESPWiFiSensing",
    cg.Component,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ESPWiFiSensing),
        cv.Optional(CONF_GAIN_COMPENSATION, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_gain_compensation_enabled(config[CONF_GAIN_COMPENSATION]))
