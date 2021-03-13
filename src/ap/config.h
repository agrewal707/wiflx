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

#ifndef WIFLX_AP_CONFIG_H
#define WIFLX_AP_CONFIG_H

#include <common/config.h>

namespace wiflx {
namespace ap {

struct config
{
  struct mac
  {
    mac () :
      inactivity_timeout (5),
      p_ack_timeout (50000),
      d_ack_timeout (50000),
      ra_ack_timeout (50000),
      max_poll_retries (3),
      max_data_retries (3),
      rm_airtime_quantum (3028),
      b_poll_airtime_quantum (30280)
    {}

    uint8_t inactivity_timeout;
    uint32_t p_ack_timeout;
    uint32_t d_ack_timeout;
    uint32_t ra_ack_timeout;
    uint8_t max_poll_retries;
    uint8_t max_data_retries;
    int32_t rm_airtime_quantum;
    int32_t b_poll_airtime_quantum;

  };
  struct l2iwf
  {
    std::string devname;
  };

  common::config::radio m_radio;
  common::config::ofdm m_ofdm;
  common::config::fq_codel m_fq_codel;
  common::config::codel m_codel;
  mac m_mac;
  l2iwf m_l2iwf;
};

} // namespace ap
} // namespace wiflx

#endif // WIFLX_AP_CONFIG_H
