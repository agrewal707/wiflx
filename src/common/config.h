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

#ifndef WIFLX_COMMON_CONFIG_H
#define WIFLX_COMMON_CONFIG_H

#include <liquid/liquid.h>
#include <string>
#include <cstdlib>
#include <vector>

namespace wiflx {
namespace common {
namespace config {

struct radio
{
  std::string driver;
  std::string uri;
  float rxfreq;
  float txfreq;
  float rxgain;
  float txgain;
  float sampling_frequency;
  float rx_analog_bandwidth;
  float tx_analog_bandwidth;
  std::string fir_filter_file;
  size_t rx_buf_size;
  size_t rx_buf_count;
  size_t tx_buf_size;
  size_t tx_buf_count;
};

struct ofdm
{
  unsigned int M = 64;          // number of subcarriers
  unsigned int cp_len = 4;      // cyclic prefix length
  unsigned int taper_len = 2;   // taper length
  std::vector<unsigned char> p;
  std::string ms;   // payload modulation scheme
  std::string fec0; // inner FEC scheme
  std::string fec1; // outer FEC scheme
  std::string check; // data validity check
};

struct sc
{
  std::string ms;    // payload modulation scheme
  std::string fec0;  // inner FEC scheme
  std::string fec1;  // outer FEC scheme
  std::string check; // data validity check
};

struct fq_codel
{
  fq_codel():
    quantum (0),
    packet_limit (0),
    batch_drop_size (0),
    max_flows (0),
    max_aggregate_size (0)
  {}

  uint32_t quantum;
  uint32_t packet_limit;
  uint32_t batch_drop_size;
  uint32_t max_flows;
  uint32_t max_aggregate_size;
};

struct codel
{
  codel ():
    max_size (0),
    min_bytes (0),
    interval (0),
    target (0)
  {}

  uint32_t max_size;
  uint32_t min_bytes;
  uint32_t interval;
  uint32_t target;
};

} // namespace config
} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_CONFIG_H
