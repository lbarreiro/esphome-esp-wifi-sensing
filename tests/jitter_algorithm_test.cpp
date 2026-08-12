#include <cassert>
#include <cstdint>

#include "../components/esp_wifi_sensing/jitter_algorithm.h"

using esphome::esp_wifi_sensing::CsiPacket;
using esphome::esp_wifi_sensing::JitterAlgorithm;

static CsiPacket packet(const int8_t *bytes, uint16_t len) {
  CsiPacket p{};
  p.raw_bytes = bytes;
  p.len = len;
  return p;
}

int main() {
  JitterAlgorithm algorithm;

  const int8_t a[] = {10, -10, 20, -20};
  const int8_t same[] = {10, -10, 20, -20};
  const int8_t changed[] = {20, -20, 40, -40};

  assert(algorithm.process(packet(a, 4)) == 0);
  assert(algorithm.process(packet(same, 4)) == 0);

  const uint32_t jitter = algorithm.process(packet(changed, 4));
  assert(jitter == 3333);

  algorithm.reset();
  assert(algorithm.process(packet(changed, 4)) == 0);

  return 0;
}
