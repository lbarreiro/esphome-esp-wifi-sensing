#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "csi_parser.h"

namespace esphome {
namespace esp_wifi_sensing {

class VarianceAlgorithm {
 public:
  uint32_t process(const ParsedCsiPacket &packet);

 private:
  static constexpr uint8_t kWindowSize = 32;

  struct SampleWindow {
    std::array<uint16_t, kWindowSize> values{};
    uint8_t next{0};
    uint8_t count{0};
  };

  std::vector<SampleWindow> sample_windows_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
