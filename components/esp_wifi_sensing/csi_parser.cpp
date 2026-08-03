#include "csi_parser.h"

#include <cmath>

namespace esphome {
namespace esp_wifi_sensing {

namespace {

bool is_ht_packet(const wifi_pkt_rx_ctrl_t *rx_ctrl) {
  return rx_ctrl != nullptr && rx_ctrl->sig_mode != 0;
}

bool is_40mhz_packet(const wifi_pkt_rx_ctrl_t *rx_ctrl) {
  return rx_ctrl != nullptr && rx_ctrl->cwb != 0;
}

bool is_stbc_packet(const wifi_pkt_rx_ctrl_t *rx_ctrl) {
  return rx_ctrl != nullptr && rx_ctrl->stbc != 0;
}

int secondary_channel(const wifi_pkt_rx_ctrl_t *rx_ctrl) {
  if (rx_ctrl == nullptr) {
    return 0;
  }
  return rx_ctrl->secondary_channel;
}

}  // namespace

const ParsedCsiPacket &CsiParser::parse(const CsiPacket &packet) {
  this->reset_();

  if (packet.raw_bytes == nullptr || packet.len < 2) {
    return this->parsed_packet_;
  }

  this->parsed_packet_.first_word_invalid = packet.first_word_invalid;

  const bool ht = is_ht_packet(packet.rx_ctrl);
  const bool cwb40 = is_40mhz_packet(packet.rx_ctrl);
  const bool stbc = is_stbc_packet(packet.rx_ctrl);
  const int secondary = secondary_channel(packet.rx_ctrl);
  const bool secondary_below = secondary == WIFI_SECOND_CHAN_BELOW;
  const bool secondary_above = secondary == WIFI_SECOND_CHAN_ABOVE;

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
