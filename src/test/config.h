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

#ifndef WIFLX_TEST_CONFIG_H
#define WIFLX_TEST_CONFIG_H

#include <common/config.h>

namespace wiflx {
namespace test {

struct config
{
  enum Type
  {
    CW_PHY_TEST,
    OFDM_PHY_TEST,
    SC_PHY_TEST,
  };

  struct cw_test
  {
    float tone_freq;
  };

  struct ofdm_test
  {
    bool tx_enable;
    bool rx_enable;
    int sdu_size;
    int num_packets;
    int tx_interval;
    common::config::ofdm ofdm;
  };

  struct sc_test
  {
    bool tx_enable;
    bool rx_enable;
    int sdu_size;
    int num_packets;
    int tx_interval;
    common::config::sc sc;
  };

  Type m_type;
  common::config::radio m_radio;
  cw_test m_cw_test;
  ofdm_test m_ofdm_test;
  sc_test m_sc_test;
};

} // namespace test
} // namespace wiflx

#endif // WIFLX_TEST_CONFIG_H
