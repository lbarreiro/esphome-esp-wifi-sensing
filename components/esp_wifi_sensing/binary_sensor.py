import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_MOTION

from . import CONF_ESP_WIFI_SENSING_ID, ESP_WIFI_SENSING_COMPONENT_SCHEMA

CONF_MOTION = "motion"

DEPENDENCIES = ["esp_wifi_sensing"]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
            cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_MOTION,
            ),
        }
    ).extend(ESP_WIFI_SENSING_COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_MOTION),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ESP_WIFI_SENSING_ID])

    if CONF_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MOTION])
        cg.add(parent.set_motion_binary_sensor(sens))
