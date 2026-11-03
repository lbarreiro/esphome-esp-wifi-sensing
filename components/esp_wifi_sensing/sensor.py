import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from . import CONF_ESP_WIFI_SENSING_ID, ESP_WIFI_SENSING_COMPONENT_SCHEMA

CONF_ULTIMA_MINUTOS = "ultima_minutos"
CONF_PORTADORA_MAIS_FRACA = "portadora_mais_fraca"

DEPENDENCIES = ["esp_wifi_sensing"]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_ULTIMA_MINUTOS): sensor.sensor_schema(
                unit_of_measurement="min",
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_PORTADORA_MAIS_FRACA): sensor.sensor_schema(
                accuracy_decimals=0,
            ),
        }
    ).extend(ESP_WIFI_SENSING_COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_ULTIMA_MINUTOS, CONF_PORTADORA_MAIS_FRACA),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ESP_WIFI_SENSING_ID])

    if CONF_ULTIMA_MINUTOS in config:
        last = await sensor.new_sensor(config[CONF_ULTIMA_MINUTOS])
        cg.add(parent.set_ultima_minutos_sensor(last))

    if CONF_PORTADORA_MAIS_FRACA in config:
        weak = await sensor.new_sensor(config[CONF_PORTADORA_MAIS_FRACA])
        cg.add(parent.set_portadora_mais_fraca_sensor(weak))
