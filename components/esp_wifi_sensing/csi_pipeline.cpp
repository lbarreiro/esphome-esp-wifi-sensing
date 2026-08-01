#include "csi_pipeline.h"

namespace esphome {
namespace esp_wifi_sensing {

CsiPipeline::CsiPipeline() = default;

void CsiPipeline::process_packet(const CsiPacket &packet) {
  latest_len_ = packet.len;
  has_new_sample_ = true;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
