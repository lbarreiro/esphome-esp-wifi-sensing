# ESPHome Wi-Fi Sensing Architecture

## Folder tree

```text
components/
└── esp_wifi_sensing/
    ├── __init__.py
    ├── wifi_sensing.h
    ├── wifi_sensing.cpp
    ├── algorithms/
    │   └── threshold_algorithm.h
    ├── driver/
    │   ├── espidf_csi_driver.h
    │   └── espidf_csi_driver.cpp
    ├── features/
    │   └── empty_feature.h
    ├── models/
    │   └── csi_packet.h
    ├── pipeline/
    │   ├── csi_pipeline.h
    │   └── csi_pipeline.cpp
    ├── preprocessing/
    │   └── empty_preprocessor.h
    └── sensors/
        └── csi_sensor.h
```

## Module overview

The root component entry point is the ESPHome integration in [components/esp_wifi_sensing/wifi_sensing.h](components/esp_wifi_sensing/wifi_sensing.h) and [components/esp_wifi_sensing/wifi_sensing.cpp](components/esp_wifi_sensing/wifi_sensing.cpp), which owns the component lifecycle, CSI callback wiring, periodic logging, and the existing metric calculation logic. The driver layer provides the boundary to ESP-IDF CSI APIs and is responsible for enabling CSI capture and registering the callback that receives raw CSI packets from the Wi-Fi stack. The pipeline layer coordinates the end-to-end flow from incoming CSI data to the current metric output while preserving the existing behavior of the original component. The models layer defines the packet container used to carry CSI data and related state between the different stages of the pipeline. The algorithms layer holds the current decision logic used to transform a CSI packet into the metric that is reported by the component. The preprocessing and features layers are scaffolded to support future processing stages, while the sensors layer is the output-facing boundary for exposing processed information.

## Processing pipeline

```text
CSI callback
    ↓
ESP-IDF driver
    ↓
CSI packet model
    ↓
Pipeline coordinator
    ↓
Threshold algorithm
    ↓
Existing metric reporting
```

## Class responsibilities

- ESPWiFiSensing: Main ESPHome component class that manages setup, loop execution, configuration, callback registration, and the existing reporting state.
- EspIdfCsiDriver: Low-level integration class that interacts with ESP-IDF CSI APIs to start CSI capture and register the callback entry point.
- CsiPipeline: Central processing coordinator that receives CSI packets, routes them through the current processing stages, and preserves the component’s existing runtime behavior.
- CsiPacket: Data container representing a CSI packet and the fields needed to carry information between pipeline stages.
- ThresholdAlgorithm: Current algorithm stage that performs the existing threshold-style metric transformation while preserving the prior logic.
- EmptyPreprocessor: Placeholder preprocessing stage for future data-cleaning or normalization work.
- EmptyFeature: Placeholder feature-extraction stage for future derived features.
- CsiSensor: Placeholder output stage for future sensor or reporting integration.

## Placeholder modules for future work

The following modules are intentionally placeholders and do not yet implement production behavior:

- [components/esp_wifi_sensing/preprocessing/empty_preprocessor.h](components/esp_wifi_sensing/preprocessing/empty_preprocessor.h)
- [components/esp_wifi_sensing/features/empty_feature.h](components/esp_wifi_sensing/features/empty_feature.h)
- [components/esp_wifi_sensing/sensors/csi_sensor.h](components/esp_wifi_sensing/sensors/csi_sensor.h)

## Modules that already contain production code

The following modules already contain the functional implementation used by the current component:

- [components/esp_wifi_sensing/wifi_sensing.h](components/esp_wifi_sensing/wifi_sensing.h)
- [components/esp_wifi_sensing/wifi_sensing.cpp](components/esp_wifi_sensing/wifi_sensing.cpp)
- [components/esp_wifi_sensing/driver/espidf_csi_driver.h](components/esp_wifi_sensing/driver/espidf_csi_driver.h)
- [components/esp_wifi_sensing/driver/espidf_csi_driver.cpp](components/esp_wifi_sensing/driver/espidf_csi_driver.cpp)
- [components/esp_wifi_sensing/pipeline/csi_pipeline.h](components/esp_wifi_sensing/pipeline/csi_pipeline.h)
- [components/esp_wifi_sensing/pipeline/csi_pipeline.cpp](components/esp_wifi_sensing/pipeline/csi_pipeline.cpp)
- [components/esp_wifi_sensing/models/csi_packet.h](components/esp_wifi_sensing/models/csi_packet.h)
- [components/esp_wifi_sensing/algorithms/threshold_algorithm.h](components/esp_wifi_sensing/algorithms/threshold_algorithm.h)
