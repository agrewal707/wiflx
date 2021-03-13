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

#ifndef WIFLX_RM_CONFIG_JSON_H
#define WIFLX_RM_CONFIG_JSON_H

#include <nlohmann/json.hpp>
#include <common/config_json.h>
#include <rm/config.h>

namespace wiflx {
namespace rm {

using json = nlohmann::json;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::mac,
  rid,
  rlf_timeout,
  ack_timeout,
  max_data_retries,
  cw_min,
  cw_max
)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::l2iwf,
  devname
)

//
// config
//
void to_json(json &j, const config &c) {
	j = json
	{
		{"radio", c.m_radio},
		{"ofdm", c.m_ofdm},
    {"fq_codel", c.m_fq_codel},
    {"codel", c.m_codel},
		{"mac", c.m_mac},
    {"l2iwf", c.m_l2iwf},
	};
}

void from_json(const json &j, config &c)
{
	j.at("radio").get_to(c.m_radio);
	j.at("ofdm").get_to(c.m_ofdm);
  j.at("fq_codel").get_to(c.m_fq_codel);
  j.at("codel").get_to(c.m_codel);
	j.at("mac").get_to(c.m_mac);
  j.at("l2iwf").get_to(c.m_l2iwf);
}

} // namespace rm
} // namespace wiflx

#endif // WIFLX_RM_CONFIG_JSON_H
