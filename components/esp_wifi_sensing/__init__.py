import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_ALGORITHM = "algorithm"
CONF_GAIN_COMPENSATION = "gain_compensation"
CONF_THRESHOLD = "threshold"
CONF_DEBOUNCE = "debounce"
CONF_ESP_WIFI_SENSING_ID = "esp_wifi_sensing_id"

DEPENDENCIES = ["wifi"]

esp_wifi_sensing_ns = cg.esphome_ns.namespace("esp_wifi_sensing")

ESPWiFiSensing = esp_wifi_sensing_ns.class_(
    "ESPWiFiSensing",
    cg.Component,
)

CsiAlgorithm = esp_wifi_sensing_ns.enum("CsiAlgorithm", is_class=True)
ALGORITHMS = {
    "absolute_sum": CsiAlgorithm.ABSOLUTE_SUM,
    "variance": CsiAlgorithm.VARIANCE,
}

ESP_WIFI_SENSING_COMPONENT_SCHEMA = {
    cv.GenerateID(CONF_ESP_WIFI_SENSING_ID): cv.use_id(ESPWiFiSensing),
}


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ESPWiFiSensing),
        cv.Optional(CONF_ALGORITHM, default="absolute_sum"): cv.enum(
            ALGORITHMS, lower=True
        ),
        cv.Optional(CONF_GAIN_COMPENSATION, default=False): cv.boolean,
        cv.Optional(CONF_THRESHOLD, default=6000): cv.uint32_t,
        cv.Optional(CONF_DEBOUNCE, default=2): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_algorithm(config[CONF_ALGORITHM]))
    cg.add(var.set_gain_compensation_enabled(config[CONF_GAIN_COMPENSATION]))
    cg.add(var.set_motion_threshold(config[CONF_THRESHOLD]))
    cg.add(var.set_motion_debounce(config[CONF_DEBOUNCE]))
    if config[CONF_GAIN_COMPENSATION]:
        cg.add_define("USE_ESP_WIFI_SENSING_GAIN_COMPENSATION")
