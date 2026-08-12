import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import ESPWiFiSensing

CONF_ESP_WIFI_SENSING_ID = "esp_wifi_sensing_id"
CONF_MOTION = "motion"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ESP_WIFI_SENSING_ID): cv.use_id(ESPWiFiSensing),
        cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ESP_WIFI_SENSING_ID])

    if CONF_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MOTION])
        cg.add(parent.set_motion_binary_sensor(sens))
