# ESPHome Wi-Fi Sensing Architecture

## Folder tree

```text
components/
└── esp_wifi_sensing/
    ├── __init__.py
    ├── csi_packet.h
    ├── csi_parser.h
    ├── csi_parser.cpp
    ├── csi_pipeline.h
    ├── csi_pipeline.cpp
    ├── csi_sensor.h
    ├── empty_feature.h
    ├── empty_preprocessor.h
    ├── espidf_csi_driver.h
    ├── espidf_csi_driver.cpp
    ├── gain_compensation_preprocessor.h
    ├── gain_compensation_preprocessor.cpp
    ├── threshold_algorithm.h
    ├── variance_algorithm.h
    ├── variance_algorithm.cpp
    ├── wifi_sensing.h
    └── wifi_sensing.cpp
```

## Module overview

The root component entry point is the ESPHome integration in [components/esp_wifi_sensing/wifi_sensing.h](components/esp_wifi_sensing/wifi_sensing.h) and [components/esp_wifi_sensing/wifi_sensing.cpp](components/esp_wifi_sensing/wifi_sensing.cpp), which owns the component lifecycle, CSI callback wiring, periodic logging, and the existing metric calculation logic. The driver layer provides the boundary to ESP-IDF CSI APIs and is responsible for enabling CSI capture and registering the callback that receives raw CSI packets from the Wi-Fi stack. The pipeline layer coordinates the end-to-end flow from incoming CSI data through gain compensation and CSI parsing while preserving the existing raw-byte algorithm behavior. The packet model carries raw CSI bytes and receive metadata, and the parser exposes reusable structured OFDM subcarriers containing subcarrier index, I/Q components, amplitude, and power for future algorithms. The algorithms layer still holds the current decision logic used to transform a CSI packet into the metric that is reported by the component.

## Processing pipeline

```text
CSI callback
    ↓
Raw CSI packet model
    ↓
Gain compensation preprocessor
    ↓
CSI parser
    ↓
Existing algorithm selection
    ↓
Existing metric reporting
```

## Class responsibilities

- ESPWiFiSensing: Main ESPHome component class that manages setup, loop execution, configuration, callback registration, and the existing reporting state.
- EspIdfCsiDriver: Low-level integration class that interacts with ESP-IDF CSI APIs to start CSI capture and register the callback entry point.
- CsiPipeline: Central processing coordinator that receives CSI packets, applies gain compensation, parses structured subcarriers, and preserves the component’s existing runtime behavior.
- CsiPacket: Data container representing raw CSI bytes, receive metadata, and the ESP-IDF `first_word_invalid` flag needed by downstream stages.
- CsiParser: Reusable parsing layer that maps supported ESP32-C6 CSI byte layouts into valid OFDM subcarriers, excluding null and guard carriers and exposing subcarrier index, I component, Q component, amplitude, and power. The current parser intentionally supports only layouts that can be identified without unavailable RX metadata.
- ThresholdAlgorithm: Current algorithm stage that performs the existing threshold-style metric transformation while preserving the prior logic.
- VarianceAlgorithm: Current raw-byte temporal variance metric implementation.
- GainCompensationPreprocessor: Optional preprocessing stage that compensates raw CSI bytes for RX gain changes before parsing or existing metric calculation.

## Modules that already contain production code

The following modules already contain the functional implementation used by the current component:

- [components/esp_wifi_sensing/wifi_sensing.h](components/esp_wifi_sensing/wifi_sensing.h)
- [components/esp_wifi_sensing/wifi_sensing.cpp](components/esp_wifi_sensing/wifi_sensing.cpp)
- [components/esp_wifi_sensing/espidf_csi_driver.h](components/esp_wifi_sensing/espidf_csi_driver.h)
- [components/esp_wifi_sensing/espidf_csi_driver.cpp](components/esp_wifi_sensing/espidf_csi_driver.cpp)
- [components/esp_wifi_sensing/csi_pipeline.h](components/esp_wifi_sensing/csi_pipeline.h)
- [components/esp_wifi_sensing/csi_pipeline.cpp](components/esp_wifi_sensing/csi_pipeline.cpp)
- [components/esp_wifi_sensing/csi_packet.h](components/esp_wifi_sensing/csi_packet.h)
- [components/esp_wifi_sensing/csi_parser.h](components/esp_wifi_sensing/csi_parser.h)
- [components/esp_wifi_sensing/csi_parser.cpp](components/esp_wifi_sensing/csi_parser.cpp)
- [components/esp_wifi_sensing/threshold_algorithm.h](components/esp_wifi_sensing/threshold_algorithm.h)
- [components/esp_wifi_sensing/variance_algorithm.h](components/esp_wifi_sensing/variance_algorithm.h)
- [components/esp_wifi_sensing/variance_algorithm.cpp](components/esp_wifi_sensing/variance_algorithm.cpp)
- [components/esp_wifi_sensing/gain_compensation_preprocessor.h](components/esp_wifi_sensing/gain_compensation_preprocessor.h)
- [components/esp_wifi_sensing/gain_compensation_preprocessor.cpp](components/esp_wifi_sensing/gain_compensation_preprocessor.cpp)
