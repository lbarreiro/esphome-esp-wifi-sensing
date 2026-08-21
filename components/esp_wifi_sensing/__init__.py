import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_ALGORITHM = "algorithm"
CONF_GAIN_COMPENSATION = "gain_compensation"
CONF_ADAPTIVE_THRESHOLD = "adaptive_threshold"
CONF_THRESHOLD = "threshold"
CONF_SIGMA_MULTIPLIER = "sigma_multiplier"
CONF_BASELINE_RISE_TIME = "baseline_rise_time"
CONF_BASELINE_FALL_TIME = "baseline_fall_time"
CONF_LEARNING_DELAY = "learning_delay"
CONF_DEBOUNCE = "debounce"
CONF_WARMUP_TIME = "warmup_time"
CONF_MOTION_HOLD_TIME = "motion_hold_time"
CONF_PERSISTENCE = "persistence"
CONF_MOTION_SENSITIVITY = "motion_sensitivity"
CONF_ACTIVE_JITTER_MIN = "active_jitter_min"
CONF_ACTIVE_FILTER_MS = "active_filter_ms"
CONF_ENTER_MULTIPLIER = "enter_multiplier"

DEPENDENCIES = ["wifi"]

esp_wifi_sensing_ns = cg.esphome_ns.namespace("esp_wifi_sensing")
ESPWiFiSensing = esp_wifi_sensing_ns.class_("ESPWiFiSensing", cg.Component)
CsiAlgorithm = esp_wifi_sensing_ns.enum("CsiAlgorithm", is_class=True)
ALGORITHMS = {
    "absolute_sum": CsiAlgorithm.ABSOLUTE_SUM,
    "variance": CsiAlgorithm.VARIANCE,
    "amplitude": CsiAlgorithm.AMPLITUDE,
    "jitter": CsiAlgorithm.JITTER,
    "esp_radar": CsiAlgorithm.ESP_RADAR,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ESPWiFiSensing),
        cv.Optional(CONF_ALGORITHM, default="absolute_sum"): cv.enum(ALGORITHMS, lower=True),
        cv.Optional(CONF_GAIN_COMPENSATION, default=False): cv.boolean,
        cv.Optional(CONF_ADAPTIVE_THRESHOLD, default=True): cv.boolean,
        cv.Optional(CONF_THRESHOLD, default=6): cv.positive_int,
        cv.Optional(CONF_SIGMA_MULTIPLIER, default=3.0): cv.float_,
        cv.Optional(CONF_BASELINE_RISE_TIME, default="10min"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_BASELINE_FALL_TIME, default="60min"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_LEARNING_DELAY, default="60s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_DEBOUNCE, default=2): cv.positive_int,
        cv.Optional(CONF_WARMUP_TIME, default="60s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_MOTION_HOLD_TIME, default="60s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_PERSISTENCE, default=7): cv.int_range(min=1, max=10),
        cv.Optional(CONF_MOTION_SENSITIVITY, default=0.5): cv.float_range(min=0.05, max=1.0),
        cv.Optional(CONF_ACTIVE_JITTER_MIN, default=0.05): cv.float_range(min=0.0, max=1.0),
        cv.Optional(CONF_ACTIVE_FILTER_MS, default="500ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ENTER_MULTIPLIER, default=1.2): cv.float_range(min=1.0, max=5.0),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_algorithm(config[CONF_ALGORITHM]))
    cg.add(var.set_gain_compensation_enabled(config[CONF_GAIN_COMPENSATION]))
    cg.add(var.set_adaptive_threshold_enabled(config[CONF_ADAPTIVE_THRESHOLD]))
    cg.add(var.set_fixed_threshold(config[CONF_THRESHOLD]))
    cg.add(var.set_sigma_multiplier(config[CONF_SIGMA_MULTIPLIER]))
    cg.add(var.set_baseline_rise_time(config[CONF_BASELINE_RISE_TIME].total_milliseconds))
    cg.add(var.set_baseline_fall_time(config[CONF_BASELINE_FALL_TIME].total_milliseconds))
    cg.add(var.set_learning_delay(config[CONF_LEARNING_DELAY].total_milliseconds))
    cg.add(var.set_debounce(config[CONF_DEBOUNCE]))
    cg.add(var.set_warmup_time(config[CONF_WARMUP_TIME].total_milliseconds))
    cg.add(var.set_motion_hold_time(config[CONF_MOTION_HOLD_TIME].total_milliseconds))
    cg.add(var.set_persistence_samples(config[CONF_PERSISTENCE]))
    cg.add(var.set_motion_sensitivity(config[CONF_MOTION_SENSITIVITY]))
    cg.add(var.set_active_jitter_min(config[CONF_ACTIVE_JITTER_MIN]))
    cg.add(var.set_active_filter_ms(config[CONF_ACTIVE_FILTER_MS].total_milliseconds))
    cg.add(var.set_enter_multiplier(config[CONF_ENTER_MULTIPLIER]))
    if config[CONF_GAIN_COMPENSATION]:
        cg.add_define("USE_ESP_WIFI_SENSING_GAIN_COMPENSATION")
