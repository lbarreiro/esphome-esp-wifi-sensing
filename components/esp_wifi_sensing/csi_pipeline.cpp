#include "csi_pipeline.h"

namespace esphome {
namespace esp_wifi_sensing {

CsiPipeline::CsiPipeline() = default;

void CsiPipeline::process_packet(const CsiPacket &packet) {
  CsiPacket processed_packet{};
  gain_compensation_preprocessor_.process(packet, processed_packet);

  latest_len_ = processed_packet.len;
  has_new_sample_ = true;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
