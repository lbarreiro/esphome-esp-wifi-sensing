#pragma once

#include <cstdint>

namespace esphome {
namespace esp_wifi_sensing {

class DiagnosticPublishRateLimiter {
 public:
  static constexpr uint32_t kPublishIntervalMs = 1000;

  bool should_publish(uint32_t now_ms) {
    if (!this->has_published_) {
      this->last_publish_ms_ = now_ms;
      this->has_published_ = true;
      return true;
    }

    if (now_ms - this->last_publish_ms_ < kPublishIntervalMs) {
      return false;
    }

    this->last_publish_ms_ = now_ms;
    return true;
  }

 protected:
  bool has_published_{false};
  uint32_t last_publish_ms_{0};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
