# ESPHome ESP Wi-Fi Sensing

External ESPHome component for collecting ESP32 Wi-Fi CSI (Channel State Information) and publishing runtime CSI diagnostics and motion state.

## Basic configuration

The default motion decision layer is adaptive. The component continuously learns the normal background while the room is unoccupied. When motion is detected, learning freezes so the moving person is not absorbed into the baseline. After motion returns to off, learning resumes only after `learning_delay`. This lets the adaptive threshold follow slow environmental changes automatically without reacting to short motion events.

```yaml
external_components:
  - source:
      type: local
      path: ..

esp_wifi_sensing:
  id: wifi_sensing
  algorithm: variance
  debounce: 2
  adaptive_threshold: true
  sigma_multiplier: 4.0
  baseline_rise_time: 30min
  baseline_fall_time: 30min
  learning_delay: 60s
```

The adaptive threshold is calculated from the existing 5-second `variation_avg` statistics window:

```text
adaptive_threshold = baseline_mean + sigma_multiplier × baseline_stddev
```

`baseline_rise_time` is used when `variation_avg` is above the baseline mean. `baseline_fall_time` is used when `variation_avg` is below the baseline mean. Both values are approximate EMA time constants, so baseline changes are smooth and continuous.

## Fixed-threshold compatibility

Set `adaptive_threshold: false` to preserve the previous fixed-threshold behavior exactly. In this mode `variation_avg` is compared directly to `threshold`, using the existing debounce behavior.

```yaml
esp_wifi_sensing:
  id: wifi_sensing
  algorithm: variance
  threshold: 6000
  debounce: 2
  adaptive_threshold: false
```

## Optional diagnostic sensors

The component can publish its existing 5-second diagnostic statistics directly as Home Assistant sensors. These sensors are optional; if the `sensor:` platform is omitted, the component only logs the same statistics as before.

The published values are the already-calculated runtime metrics plus optional adaptive-baseline diagnostics:

- `metric`: the latest CSI metric from the selected algorithm.
- `variation_avg`: the average CSI variation for the current 5-second statistics window.
- `baseline_mean`: the learned normal background mean.
- `baseline_stddev`: the learned normal background standard deviation.
- `adaptive_threshold`: the active adaptive motion threshold.

```yaml
esp_wifi_sensing:
  id: wifi_sensing
  algorithm: variance
  debounce: 2
  adaptive_threshold: true
  sigma_multiplier: 4.0
  baseline_rise_time: 30min
  baseline_fall_time: 30min
  learning_delay: 60s

sensor:
  - platform: esp_wifi_sensing
    esp_wifi_sensing_id: wifi_sensing
    metric:
      name: "CSI Metric"
    variation_avg:
      name: "CSI Variation Avg"
    baseline_mean:
      name: "Baseline Mean"
    baseline_stddev:
      name: "Baseline StdDev"
    adaptive_threshold:
      name: "Adaptive Threshold"
```

All diagnostic sensors are published when the component updates its existing 5-second statistics report.

## Optional motion binary sensor

The motion binary sensor uses the existing `variation_avg` value from each 5-second statistics window. With adaptive thresholding enabled, motion turns on after `variation_avg` is above the adaptive threshold for `debounce` consecutive reports, and turns off immediately when `variation_avg` is less than or equal to the adaptive threshold. With adaptive thresholding disabled, the same behavior uses the fixed `threshold` value instead.

```yaml
binary_sensor:
  - platform: esp_wifi_sensing
    esp_wifi_sensing_id: wifi_sensing
    motion:
      name: "CSI Motion"
```
