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

#ifndef WIFLX_RM_CONFIG_H
#define WIFLX_RM_CONFIG_H

#include <common/config.h>

namespace wiflx {
namespace rm {

struct config
{
  struct mac
  {
    mac () :
      rid (1),
      rlf_timeout (5),
      ack_timeout (50000),
      max_data_retries (3),
      cw_min (1),
      cw_max (6)
    {}

    uint16_t rid;
    uint8_t rlf_timeout;
    uint32_t ack_timeout;
    uint8_t max_data_retries;
    uint8_t cw_min;
    uint8_t cw_max;
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

} // namespace rm
} // namespace wiflx

#endif // WIFLX_RM_CONFIG_H
