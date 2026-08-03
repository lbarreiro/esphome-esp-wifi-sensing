#pragma once

#include <cstdint>
#include <cstddef>

#include "csi_packet.h"

namespace esphome {
namespace esp_wifi_sensing {

struct CsiSubcarrier {
  int16_t index{0};
  int8_t i{0};
  int8_t q{0};
  float amplitude{0.0f};
  uint16_t power{0};
};

struct ParsedCsiPacket {
  static constexpr size_t kMaxSubcarriers = 320;

  CsiSubcarrier subcarriers[kMaxSubcarriers]{};
  size_t count{0};
  bool truncated{false};
  bool first_word_invalid{false};
};

class CsiParser {
 public:
  const ParsedCsiPacket &parse(const CsiPacket &packet);
  const ParsedCsiPacket &latest_packet() const { return parsed_packet_; }

 private:
  enum class PhyMode : uint8_t {
    NON_HT,
    HT20,
    HT40,
  };

  void reset_();
  void parse_range_(const CsiPacket &packet, uint16_t &offset, int16_t start, int16_t end, PhyMode phy_mode);
  bool append_subcarrier_(const CsiPacket &packet, uint16_t offset, int16_t index, PhyMode phy_mode);
  bool is_valid_subcarrier_(int16_t index, PhyMode phy_mode) const;

  ParsedCsiPacket parsed_packet_{};
};

}  // namespace esp_wifi_sensing
}  // namespace esphome
