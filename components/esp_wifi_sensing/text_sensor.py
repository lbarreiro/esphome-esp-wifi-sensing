import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_ESP_WIFI_SENSING_ID, ESP_WIFI_SENSING_COMPONENT_SCHEMA

CONF_VIA = "via"
CONF_PORTADORAS_TOP = "portadoras_top"
CONF_TOP_CLASSIFICACAO = "top_classificacao"

DEPENDENCIES = ["esp_wifi_sensing"]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_VIA): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_PORTADORAS_TOP): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_TOP_CLASSIFICACAO): text_sensor.text_sensor_schema(),
        }
    ).extend(ESP_WIFI_SENSING_COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_VIA, CONF_PORTADORAS_TOP, CONF_TOP_CLASSIFICACAO),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ESP_WIFI_SENSING_ID])

    if CONF_VIA in config:
        via = await text_sensor.new_text_sensor(config[CONF_VIA])
        cg.add(parent.set_via_text_sensor(via))

    if CONF_PORTADORAS_TOP in config:
        top = await text_sensor.new_text_sensor(config[CONF_PORTADORAS_TOP])
        cg.add(parent.set_portadoras_top_text_sensor(top))

    if CONF_TOP_CLASSIFICACAO in config:
        ranked = await text_sensor.new_text_sensor(config[CONF_TOP_CLASSIFICACAO])
        cg.add(parent.set_top_classificacao_text_sensor(ranked))
