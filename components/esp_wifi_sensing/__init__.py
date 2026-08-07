import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_ALGORITHM = "algorithm"
CONF_GAIN_COMPENSATION = "gain_compensation"
CONF_THRESHOLD = "threshold"
CONF_DEBOUNCE = "debounce"
CONF_ADAPTIVE_THRESHOLD = "adaptive_threshold"
CONF_SIGMA_MULTIPLIER = "sigma_multiplier"
CONF_BASELINE_RISE_TIME = "baseline_rise_time"
CONF_BASELINE_FALL_TIME = "baseline_fall_time"
CONF_LEARNING_DELAY = "learning_delay"
CONF_WARMUP_TIME = "warmup_time"
CONF_MOTION_HOLD_TIME = "motion_hold_time"
CONF_STATISTICS_WINDOW = "statistics_window"
CONF_STATISTICS_UPDATE = "statistics_update"
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


def validate_statistics_timing(config):
    if (
        config[CONF_STATISTICS_UPDATE].total_milliseconds
        > config[CONF_STATISTICS_WINDOW].total_milliseconds
    ):
        raise cv.Invalid("statistics_update must be less than or equal to statistics_window")
    return config


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ESPWiFiSensing),
        cv.Optional(CONF_ALGORITHM, default="absolute_sum"): cv.enum(
            ALGORITHMS, lower=True
        ),
        cv.Optional(CONF_GAIN_COMPENSATION, default=False): cv.boolean,
        cv.Optional(CONF_THRESHOLD, default=6000): cv.uint32_t,
        cv.Optional(CONF_DEBOUNCE, default=2): cv.positive_int,
        cv.Optional(CONF_ADAPTIVE_THRESHOLD, default=True): cv.boolean,
        cv.Optional(CONF_SIGMA_MULTIPLIER, default=4.0): cv.positive_float,
        cv.Optional(CONF_BASELINE_RISE_TIME, default="30min"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_BASELINE_FALL_TIME, default="30min"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_LEARNING_DELAY, default="60s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_WARMUP_TIME, default="0s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_MOTION_HOLD_TIME, default="0s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_STATISTICS_WINDOW, default="5s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_STATISTICS_UPDATE, default="1s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA).add_extra(validate_statistics_timing)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_algorithm(config[CONF_ALGORITHM]))
    cg.add(var.set_gain_compensation_enabled(config[CONF_GAIN_COMPENSATION]))
    cg.add(var.set_motion_threshold(config[CONF_THRESHOLD]))
    cg.add(var.set_motion_debounce(config[CONF_DEBOUNCE]))
    cg.add(var.set_adaptive_threshold_enabled(config[CONF_ADAPTIVE_THRESHOLD]))
    cg.add(var.set_sigma_multiplier(config[CONF_SIGMA_MULTIPLIER]))
    cg.add(var.set_baseline_rise_time(config[CONF_BASELINE_RISE_TIME].total_milliseconds))
    cg.add(var.set_baseline_fall_time(config[CONF_BASELINE_FALL_TIME].total_milliseconds))
    cg.add(var.set_learning_delay(config[CONF_LEARNING_DELAY].total_milliseconds))
    cg.add(var.set_warmup_time(config[CONF_WARMUP_TIME].total_milliseconds))
    cg.add(var.set_motion_hold_time(config[CONF_MOTION_HOLD_TIME].total_milliseconds))
    cg.add(var.set_statistics_window(config[CONF_STATISTICS_WINDOW].total_milliseconds))
    cg.add(var.set_statistics_update(config[CONF_STATISTICS_UPDATE].total_milliseconds))
    if config[CONF_GAIN_COMPENSATION]:
        cg.add_define("USE_ESP_WIFI_SENSING_GAIN_COMPENSATION")
