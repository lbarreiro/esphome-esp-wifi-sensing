import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)

from . import CONF_ESP_WIFI_SENSING_ID, ESP_WIFI_SENSING_COMPONENT_SCHEMA

CONF_METRIC = "metric"
CONF_VARIATION_AVG = "variation_avg"
CONF_BASELINE_MEAN = "baseline_mean"
CONF_BASELINE_STDDEV = "baseline_stddev"
CONF_ADAPTIVE_THRESHOLD = "adaptive_threshold"

DEPENDENCIES = ["esp_wifi_sensing"]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
            cv.Optional(CONF_METRIC): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_VARIATION_AVG): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_BASELINE_MEAN): sensor.sensor_schema(
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_BASELINE_STDDEV): sensor.sensor_schema(
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_ADAPTIVE_THRESHOLD): sensor.sensor_schema(
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    ).extend(ESP_WIFI_SENSING_COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_METRIC, CONF_VARIATION_AVG, CONF_BASELINE_MEAN, CONF_BASELINE_STDDEV, CONF_ADAPTIVE_THRESHOLD),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ESP_WIFI_SENSING_ID])

    if CONF_METRIC in config:
        sens = await sensor.new_sensor(config[CONF_METRIC])
        cg.add(parent.set_metric_sensor(sens))

    if CONF_VARIATION_AVG in config:
        sens = await sensor.new_sensor(config[CONF_VARIATION_AVG])
        cg.add(parent.set_variation_avg_sensor(sens))

    if CONF_BASELINE_MEAN in config:
        sens = await sensor.new_sensor(config[CONF_BASELINE_MEAN])
        cg.add(parent.set_baseline_mean_sensor(sens))

    if CONF_BASELINE_STDDEV in config:
        sens = await sensor.new_sensor(config[CONF_BASELINE_STDDEV])
        cg.add(parent.set_baseline_stddev_sensor(sens))

    if CONF_ADAPTIVE_THRESHOLD in config:
        sens = await sensor.new_sensor(config[CONF_ADAPTIVE_THRESHOLD])
        cg.add(parent.set_adaptive_threshold_sensor(sens))
