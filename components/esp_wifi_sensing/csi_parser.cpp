#include "csi_parser.h"

#include <cmath>

namespace esphome {
namespace esp_wifi_sensing {

namespace {

struct CsiRxMetadata {
  bool ht{false};
  bool cwb40{false};
  bool stbc{false};
  bool secondary_below{false};
  bool secondary_above{false};
};

CsiRxMetadata get_rx_metadata(const CsiPacket &packet) {
  CsiRxMetadata metadata{};

  if (packet.rx_ctrl == nullptr) {
    return metadata;
  }

#if CONFIG_SOC_WIFI_HE_SUPPORT
  metadata.ht = packet.rx_ctrl->cur_bb_format == RX_BB_FORMAT_HT;
  metadata.cwb40 = packet.rx_ctrl->second != WIFI_SECOND_CHAN_NONE;
  metadata.secondary_below = packet.rx_ctrl->second == WIFI_SECOND_CHAN_BELOW;
  metadata.secondary_above = packet.rx_ctrl->second == WIFI_SECOND_CHAN_ABOVE;
#else
  metadata.ht = packet.rx_ctrl->sig_mode != 0;
  metadata.cwb40 = packet.rx_ctrl->cwb != 0;
  metadata.secondary_below = packet.rx_ctrl->secondary_channel == WIFI_SECOND_CHAN_BELOW;
  metadata.secondary_above = packet.rx_ctrl->secondary_channel == WIFI_SECOND_CHAN_ABOVE;
  metadata.stbc = packet.rx_ctrl->stbc != 0;
#endif

  if (metadata.ht && !metadata.stbc) {
    if (metadata.cwb40) {
      metadata.stbc = packet.len >= 612;
    } else {
      metadata.stbc = packet.len >= 384;
    }
  }

  return metadata;
}

}  // namespace

const ParsedCsiPacket &CsiParser::parse(const CsiPacket &packet) {
  this->reset_();

  if (packet.raw_bytes == nullptr || packet.len < 2) {
    return this->parsed_packet_;
  }

  this->parsed_packet_.first_word_invalid = packet.first_word_invalid;

  const CsiRxMetadata metadata = get_rx_metadata(packet);
  const bool ht = metadata.ht;
  const bool cwb40 = metadata.cwb40;
  const bool stbc = metadata.stbc;
  const bool secondary_below = metadata.secondary_below;
  const bool secondary_above = metadata.secondary_above;

  uint16_t offset = 0;

  if (!ht) {
    if (secondary_above) {
      this->parse_range_(packet, offset, -64, -1, PhyMode::NON_HT);
    } else if (secondary_below) {
      this->parse_range_(packet, offset, 0, 63, PhyMode::NON_HT);
    } else {
      this->parse_range_(packet, offset, 0, 31, PhyMode::NON_HT);
      this->parse_range_(packet, offset, -32, -1, PhyMode::NON_HT);
    }
    return this->parsed_packet_;
  }

  PhyMode phy_mode = cwb40 ? PhyMode::HT40 : PhyMode::HT20;

  if (secondary_above) {
    this->parse_range_(packet, offset, -64, -1, phy_mode);  // LLTF
  } else if (secondary_below) {
    this->parse_range_(packet, offset, 0, 63, phy_mode);  // LLTF
  } else {
    this->parse_range_(packet, offset, 0, 31, phy_mode);  // LLTF
    this->parse_range_(packet, offset, -32, -1, phy_mode);
  }

  if (offset >= packet.len) {
    return this->parsed_packet_;
  }

  if (!cwb40) {
    this->parse_range_(packet, offset, 0, 31, phy_mode);  // HT-LTF
    this->parse_range_(packet, offset, -32, -1, phy_mode);

    if (stbc && offset < packet.len) {
      this->parse_range_(packet, offset, 0, 31, phy_mode);  // STBC-HT-LTF
      this->parse_range_(packet, offset, -32, -1, phy_mode);
    }
    return this->parsed_packet_;
  }

  if (stbc) {
    this->parse_range_(packet, offset, 0, 60, phy_mode);  // HT-LTF
    this->parse_range_(packet, offset, -60, -1, phy_mode);
  } else {
    this->parse_range_(packet, offset, 0, 63, phy_mode);  // HT-LTF
    this->parse_range_(packet, offset, -64, -1, phy_mode);
  }

  if (!stbc || offset >= packet.len) {
    return this->parsed_packet_;
  }

  this->parse_range_(packet, offset, 0, 60, phy_mode);  // STBC-HT-LTF
  this->parse_range_(packet, offset, -60, -1, phy_mode);

  return this->parsed_packet_;
}

void CsiParser::reset_() {
  this->parsed_packet_.count = 0;
  this->parsed_packet_.truncated = false;
  this->parsed_packet_.first_word_invalid = false;
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
