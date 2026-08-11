import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import STATE_CLASS_MEASUREMENT, UNIT_PERCENT

from . import ESPWiFiSensing

CONF_ESP_WIFI_SENSING_ID = "esp_wifi_sensing_id"
CONF_CSI_DELTA = "csi_delta"
CONF_CSI_TEMPORAL_MEAN = "csi_temporal_mean"
CONF_CSI_TEMPORAL_PERSISTENCE = "csi_temporal_persistence"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ESP_WIFI_SENSING_ID): cv.use_id(ESPWiFiSensing),
        cv.Optional(CONF_CSI_DELTA): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CSI_TEMPORAL_MEAN): sensor.sensor_schema(
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CSI_TEMPORAL_PERSISTENCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ESP_WIFI_SENSING_ID])

    if CONF_CSI_DELTA in config:
        sens = await sensor.new_sensor(config[CONF_CSI_DELTA])
        cg.add(parent.set_csi_delta_sensor(sens))

    if CONF_CSI_TEMPORAL_MEAN in config:
        sens = await sensor.new_sensor(config[CONF_CSI_TEMPORAL_MEAN])
        cg.add(parent.set_csi_temporal_mean_sensor(sens))

    if CONF_CSI_TEMPORAL_PERSISTENCE in config:
        sens = await sensor.new_sensor(config[CONF_CSI_TEMPORAL_PERSISTENCE])
        cg.add(parent.set_csi_temporal_persistence_sensor(sens))
