# ESPHome ESP Wi-Fi Sensing

External ESPHome component for collecting ESP32 Wi-Fi CSI (Channel State Information) and logging the existing runtime CSI statistics.

## Basic configuration

```yaml
external_components:
  - source:
      type: local
      path: ..

esp_wifi_sensing:
  id: wifi_sensing
  algorithm: variance
  threshold: 6000
  debounce: 2
```

## Optional diagnostic sensors

The component can publish its existing 5-second diagnostic statistics directly as Home Assistant sensors. These sensors are optional; if the `sensor:` platform is omitted, the component only logs the same statistics as before.

The published values are the already-calculated runtime metrics:

- `metric`: the latest CSI metric from the selected algorithm.
- `variation_avg`: the average CSI variation for the current 5-second statistics window.

```yaml
esp_wifi_sensing:
  id: wifi_sensing
  algorithm: variance
  threshold: 6000
  debounce: 2

sensor:
  - platform: esp_wifi_sensing
    esp_wifi_sensing_id: wifi_sensing
    metric:
      name: "CSI Metric"
    variation_avg:
      name: "CSI Variation Avg"
```

Both sensors are published when the component updates its existing 5-second statistics report.

## Optional motion binary sensor

The first motion decision stage uses the existing `variation_avg` value from each 5-second statistics window. Motion turns on after `variation_avg` is above `threshold` for `debounce` consecutive reports, and turns off immediately when `variation_avg` is less than or equal to `threshold`. This is a fixed-threshold stage only; adaptive thresholds are not implemented here.

```yaml
esp_wifi_sensing:
  id: wifi_sensing
  algorithm: variance
  threshold: 6000
  debounce: 2

binary_sensor:
  - platform: esp_wifi_sensing
    esp_wifi_sensing_id: wifi_sensing
    motion:
      name: "CSI Motion"
```
