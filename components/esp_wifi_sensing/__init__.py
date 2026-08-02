import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_GAIN_COMPENSATION = "gain_compensation"
CONF_DEBUG_DUMP_CSI = "debug_dump_csi"

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
        cv.Optional(CONF_DEBUG_DUMP_CSI, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_gain_compensation_enabled(config[CONF_GAIN_COMPENSATION]))
    cg.add(var.set_debug_dump_csi_enabled(config[CONF_DEBUG_DUMP_CSI]))
    if config[CONF_GAIN_COMPENSATION] or config[CONF_DEBUG_DUMP_CSI]:
        cg.add_define("USE_ESP_WIFI_SENSING_GAIN_COMPENSATION")
