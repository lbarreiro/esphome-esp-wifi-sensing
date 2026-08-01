#pragma once

#include <cstdint>

#include "esp_wifi.h"

namespace esphome {
namespace esp_wifi_sensing {

class EspIdfCsiDriver {
 public:
  bool start(void *ctx, wifi_csi_rx_cb_t callback);

 private:
  bool started_{false};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
