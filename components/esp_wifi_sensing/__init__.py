import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_ALGORITHM = "algorithm"
CONF_GAIN_COMPENSATION = "gain_compensation"
CONF_THRESHOLD = "threshold"
CONF_MOTION_HOLD_TIME = "motion_hold_time"
CONF_FILTER_WINDOW = "filter_window"
CONF_FILTER_COUNT = "filter_count"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_QUIET_BOOT_SAMPLES = "quiet_boot_samples"
CONF_QUIET_ALPHA = "quiet_alpha"
CONF_QUIET_UPDATE_LIMIT = "quiet_update_limit"
CONF_PING_INTERVAL = "ping_interval"
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
    "mvs": CsiAlgorithm.MVS,
}

ESP_WIFI_SENSING_COMPONENT_SCHEMA = {
    cv.GenerateID(CONF_ESP_WIFI_SENSING_ID): cv.use_id(ESPWiFiSensing),
}


def validate_mvs(config):
    if config[CONF_FILTER_COUNT] > config[CONF_FILTER_WINDOW]:
        raise cv.Invalid("filter_count must be <= filter_window")
    if config[CONF_PING_INTERVAL].total_milliseconds <= 80:
        raise cv.Invalid("ping_interval must be greater than the fixed 80ms ping timeout")
    return config


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ESPWiFiSensing),
        cv.Optional(CONF_ALGORITHM, default="mvs"): cv.enum(ALGORITHMS, lower=True),
        cv.Optional(CONF_GAIN_COMPENSATION, default=False): cv.boolean,
        cv.Optional(CONF_THRESHOLD, default=8.0): cv.positive_float,
        cv.Optional(CONF_MOTION_HOLD_TIME, default="120s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_FILTER_WINDOW, default=8): cv.int_range(min=1, max=64),
        cv.Optional(CONF_FILTER_COUNT, default=3): cv.int_range(min=1, max=64),
        cv.Optional(CONF_UPDATE_INTERVAL, default="100ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_QUIET_BOOT_SAMPLES, default=20): cv.int_range(min=1, max=65535),
        cv.Optional(CONF_QUIET_ALPHA, default=0.01): cv.float_range(min=0.000001, max=1.0),
        cv.Optional(CONF_QUIET_UPDATE_LIMIT, default=3.0): cv.positive_float,
        cv.Optional(CONF_PING_INTERVAL, default="100ms"): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA).add_extra(validate_mvs)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_algorithm(config[CONF_ALGORITHM]))
    cg.add(var.set_gain_compensation_enabled(config[CONF_GAIN_COMPENSATION]))
    cg.add(var.set_motion_threshold(config[CONF_THRESHOLD]))
    cg.add(var.set_motion_hold_time(config[CONF_MOTION_HOLD_TIME].total_milliseconds))
    cg.add(var.set_filter_window(config[CONF_FILTER_WINDOW]))
    cg.add(var.set_filter_count(config[CONF_FILTER_COUNT]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL].total_milliseconds))
    cg.add(var.set_quiet_boot_samples(config[CONF_QUIET_BOOT_SAMPLES]))
    cg.add(var.set_quiet_alpha(config[CONF_QUIET_ALPHA]))
    cg.add(var.set_quiet_update_limit(config[CONF_QUIET_UPDATE_LIMIT]))
    cg.add(var.set_ping_interval(config[CONF_PING_INTERVAL].total_milliseconds))
    if config[CONF_GAIN_COMPENSATION]:
        cg.add_define("USE_ESP_WIFI_SENSING_GAIN_COMPENSATION")
