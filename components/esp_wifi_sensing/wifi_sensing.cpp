#include "wifi_sensing.h"

namespace esphome {
namespace esp_wifi_sensing {

static const char *const TAG = "esp_wifi_sensing";


void ESPWiFiSensing::setup() {
  ESP_LOGI(TAG, "ESP Wi-Fi Sensing bridge starting...");
  ESP_LOGI(TAG, "STEP 2 OK - Minimal component loaded");
}


void ESPWiFiSensing::loop() {
  // Ainda não fazemos nada.
  // CSI e esp-radar entram nos próximos passos.
}


void ESPWiFiSensing::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP Wi-Fi Sensing:");
  ESP_LOGCONFIG(TAG, "  STEP 2 - Minimal component");
  ESP_LOGCONFIG(TAG, "  esp_wifi_sensing FSM: REMOVED");
  ESP_LOGCONFIG(TAG, "  CSI: NOT STARTED");
  ESP_LOGCONFIG(TAG, "  esp-radar processing: NOT STARTED");
}


}  // namespace esp_wifi_sensing
}  // namespace esphome
