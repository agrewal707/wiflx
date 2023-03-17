/*
 * Copyright (C) 2021, Ajay Pal S. Grewal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see https://opensource.org/licenses/GPL-2.0
 */

#ifndef WIFLX_COMMON_CONFIG_JSON_H
#define WIFLX_COMMON_CONFIG_JSON_H

#include <nlohmann/json.hpp>
#include <common/config.h>

namespace wiflx {
namespace common {
namespace config {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::radio,
  driver,
  uri,
  rxfreq,
  txfreq,
  rxgain,
  txgain,
  sampling_frequency,
  rx_analog_bandwidth,
  tx_analog_bandwidth,
  fir_filter_file,
  rx_buf_size,
  rx_buf_count,
  tx_buf_size,
  tx_buf_count
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::ofdm,
  M,
  cp_len,
  taper_len,
  p,
  ms,
  fec0,
  fec1,
  check
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::sc,
  ms,
  fec0,
  fec1,
  check,
  rx_D,
  rx_lo_offset_en,
  rx_lo_offset
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::fq_codel,
  quantum,
  packet_limit,
  batch_drop_size,
  max_flows,
  max_aggregate_size
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::codel,
  max_size,
  min_bytes,
  interval,
  target
)

} // namespace config
} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_CONFIG_JSON_H
