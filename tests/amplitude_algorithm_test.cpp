#include <cassert>
#include <cstdint>

#include "../components/esp_wifi_sensing/amplitude_algorithm.h"

using esphome::esp_wifi_sensing::CsiAmplitudeAlgorithm;
using esphome::esp_wifi_sensing::CsiPacket;

static CsiPacket packet(const int8_t *bytes, uint16_t len) {
  CsiPacket p{};
  p.raw_bytes = bytes;
  p.len = len;
  return p;
}

int main() {
  CsiAmplitudeAlgorithm algorithm;

  const int8_t a[] = {10, 0, 20, 0, 30, 0, 40, 0};
  const int8_t same_scale[] = {20, 0, 40, 0, 60, 0, 80, 0};
  const int8_t redistributed[] = {40, 0, 20, 0, 20, 0, 40, 0};

  assert(algorithm.process(packet(a, sizeof(a))) == 0);

  // Common-mode amplitude scaling must remain visible to preserve motion-related
  // amplitude changes rather than removing them through packet normalization.
  assert(algorithm.process(packet(same_scale, sizeof(same_scale))) > 0);

  // Redistributing energy across subcarriers must also produce a non-zero metric.
  assert(algorithm.process(packet(redistributed, sizeof(redistributed))) > 0);

  algorithm.reset();
  assert(algorithm.process(packet(a, sizeof(a))) == 0);

  return 0;
}
