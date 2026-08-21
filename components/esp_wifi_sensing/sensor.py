import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import STATE_CLASS_MEASUREMENT

from . import ESPWiFiSensing

CONF_ESP_WIFI_SENSING_ID = "esp_wifi_sensing_id"
CONF_METRIC = "metric"
CONF_BASELINE_MEAN = "baseline_mean"
CONF_BASELINE_STDDEV = "baseline_stddev"
CONF_ADAPTIVE_THRESHOLD = "adaptive_threshold"
CONF_CSI_JITTER = "csi_jitter"
CONF_CSI_ENTER_LEVEL = "csi_enter_level"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ESP_WIFI_SENSING_ID): cv.use_id(ESPWiFiSensing),
        cv.Optional(CONF_METRIC): sensor.sensor_schema(accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_BASELINE_MEAN): sensor.sensor_schema(accuracy_decimals=2, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_BASELINE_STDDEV): sensor.sensor_schema(accuracy_decimals=2, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_ADAPTIVE_THRESHOLD): sensor.sensor_schema(accuracy_decimals=2, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_CSI_JITTER): sensor.sensor_schema(accuracy_decimals=3, state_class=STATE_CLASS_MEASUREMENT),
        cv.Optional(CONF_CSI_ENTER_LEVEL): sensor.sensor_schema(accuracy_decimals=3, state_class=STATE_CLASS_MEASUREMENT),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ESP_WIFI_SENSING_ID])
    if CONF_METRIC in config:
        sens = await sensor.new_sensor(config[CONF_METRIC])
        cg.add(parent.set_metric_sensor(sens))
    if CONF_BASELINE_MEAN in config:
        sens = await sensor.new_sensor(config[CONF_BASELINE_MEAN])
        cg.add(parent.set_baseline_mean_sensor(sens))
    if CONF_BASELINE_STDDEV in config:
        sens = await sensor.new_sensor(config[CONF_BASELINE_STDDEV])
        cg.add(parent.set_baseline_stddev_sensor(sens))
    if CONF_ADAPTIVE_THRESHOLD in config:
        sens = await sensor.new_sensor(config[CONF_ADAPTIVE_THRESHOLD])
        cg.add(parent.set_adaptive_threshold_sensor(sens))
    if CONF_CSI_JITTER in config:
        sens = await sensor.new_sensor(config[CONF_CSI_JITTER])
        cg.add(parent.set_csi_jitter_sensor(sens))
    if CONF_CSI_ENTER_LEVEL in config:
        sens = await sensor.new_sensor(config[CONF_CSI_ENTER_LEVEL])
        cg.add(parent.set_csi_enter_level_sensor(sens))
