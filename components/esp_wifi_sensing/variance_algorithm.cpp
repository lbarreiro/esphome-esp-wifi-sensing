#include "variance_algorithm.h"

namespace esphome {
namespace esp_wifi_sensing {

uint32_t VarianceAlgorithm::process(const ParsedCsiPacket &packet) {
  if (packet.count == 0) {
    return 0;
  }

  if (this->sample_windows_.size() < packet.count) {
    this->sample_windows_.resize(packet.count);
  }

  uint32_t metric = 0;

  for (size_t i = 0; i < packet.count; i++) {
    SampleWindow &window = this->sample_windows_[i];

    window.values[window.next] = packet.subcarriers[i].power;
    window.next = (window.next + 1) % kWindowSize;

    if (window.count < kWindowSize) {
      window.count++;
    }

    int64_t sum = 0;
    int64_t sum_squares = 0;

    for (uint8_t j = 0; j < window.count; j++) {
      const int64_t value = static_cast<int64_t>(window.values[j]);
      sum += value;
      sum_squares += value * value;
    }

    const int64_t count = window.count;
    const int64_t numerator = (count * sum_squares) - (sum * sum);
    const uint32_t variance = static_cast<uint32_t>(
        numerator / (count * count)
    );

    metric += variance;
  }

  return metric;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
