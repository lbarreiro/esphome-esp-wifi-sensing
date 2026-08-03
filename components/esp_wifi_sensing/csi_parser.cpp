#include "csi_parser.h"

#include <cmath>

namespace esphome {
namespace esp_wifi_sensing {

namespace {

constexpr uint16_t CSI_LEN_HT20_NON_STBC = 256;

}  // namespace

const ParsedCsiPacket &CsiParser::parse(const CsiPacket &packet) {
  this->reset_();

  if (packet.raw_bytes == nullptr || packet.len < 2) {
    return this->parsed_packet_;
  }

  this->parsed_packet_.first_word_invalid = packet.first_word_invalid;

  if (packet.len != CSI_LEN_HT20_NON_STBC) {
    return this->parsed_packet_;
  }

  this->parsed_packet_.layout_supported = true;

  uint16_t offset = 0;

  this->parse_range_(packet, offset, 0, 31, PhyMode::HT20);  // LLTF
  this->parse_range_(packet, offset, -32, -1, PhyMode::HT20);
  this->parse_range_(packet, offset, 0, 31, PhyMode::HT20);  // HT-LTF
  this->parse_range_(packet, offset, -32, -1, PhyMode::HT20);

  return this->parsed_packet_;
}

void CsiParser::reset_() {
  this->parsed_packet_.count = 0;
  this->parsed_packet_.truncated = false;
  this->parsed_packet_.first_word_invalid = false;
  this->parsed_packet_.layout_supported = false;
}

void CsiParser::parse_range_(const CsiPacket &packet, uint16_t &offset, int16_t start, int16_t end, PhyMode phy_mode) {
  const int16_t step = start <= end ? 1 : -1;

  for (int16_t index = start;; index = static_cast<int16_t>(index + step)) {
    if (!this->append_subcarrier_(packet, offset, index, phy_mode)) {
      return;
    }

    offset = static_cast<uint16_t>(offset + 2);

    if (index == end) {
      return;
    }
  }
}

bool CsiParser::append_subcarrier_(const CsiPacket &packet, uint16_t offset, int16_t index, PhyMode phy_mode) {
  if (offset + 1 >= packet.len) {
    return false;
  }

  const bool invalid_first_word = packet.first_word_invalid && offset < 4;
  if (invalid_first_word || !this->is_valid_subcarrier_(index, phy_mode)) {
    return true;
  }

  if (this->parsed_packet_.count >= ParsedCsiPacket::kMaxSubcarriers) {
    this->parsed_packet_.truncated = true;
    return true;
  }

  const int8_t q = packet.raw_bytes[offset];
  const int8_t i = packet.raw_bytes[offset + 1];
  const int16_t i_value = i;
  const int16_t q_value = q;
  const uint16_t power = static_cast<uint16_t>((i_value * i_value) + (q_value * q_value));

  CsiSubcarrier &subcarrier = this->parsed_packet_.subcarriers[this->parsed_packet_.count++];
  subcarrier.index = index;
  subcarrier.i = i;
  subcarrier.q = q;
  subcarrier.power = power;
  subcarrier.amplitude = std::sqrt(static_cast<float>(power));

  return true;
}

bool CsiParser::is_valid_subcarrier_(int16_t index, PhyMode phy_mode) const {
  const int16_t abs_index = index < 0 ? static_cast<int16_t>(-index) : index;

  if (abs_index == 0) {
    return false;
  }

  switch (phy_mode) {
    case PhyMode::NON_HT:
      return abs_index <= 26;
    case PhyMode::HT20:
      return abs_index <= 28;
    case PhyMode::HT40:
      return abs_index <= 57;
  }

  return false;
}

}  // namespace esp_wifi_sensing
}  // namespace esphome
