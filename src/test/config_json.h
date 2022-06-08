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

#ifndef WIFLX_TEST_CONFIG_JSON_H
#define WIFLX_TEST_CONFIG_JSON_H

#include <nlohmann/json.hpp>
#include <common/config_json.h>
#include <test/config.h>

namespace wiflx {
namespace test {

using json = nlohmann::json;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::cw_test,
	tone_freq
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::ofdm_test,
  tx_enable,
  rx_enable,
  sdu_size,
  num_packets,
  tx_interval,
  ofdm
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  config::sc_test,
  tx_enable,
  rx_enable,
  sdu_size,
  num_packets,
  tx_interval,
  sc
)

NLOHMANN_JSON_SERIALIZE_ENUM(config::Type, {
  {config::CW_PHY_TEST, "cw_test"},
  {config::OFDM_PHY_TEST, "ofdm_test"},
  {config::SC_PHY_TEST, "sc_test"},
})

//
// config
//
void to_json(json &j, const config &c) {
	j = json
	{
    {"type", c.m_type},
    {"radio", c.m_radio},
    {"cw_test", c.m_cw_test},
    {"ofdm_test", c.m_ofdm_test},
    {"sc_test", c.m_sc_test},
  };
}

void from_json(const json &j, config &c)
{
  j.at("type").get_to(c.m_type);
  j.at("radio").get_to(c.m_radio);

  if (config::CW_PHY_TEST == c.m_type)
  {
    j.at("cw_test").get_to(c.m_cw_test);
  }
  else if (config::OFDM_PHY_TEST == c.m_type)
  {
    j.at("ofdm_test").get_to(c.m_ofdm_test);
  }
  else if (config::SC_PHY_TEST == c.m_type)
  {
    j.at("sc_test").get_to(c.m_sc_test);
  }
}

} // namespace test
} // namespace wiflx

#endif // WIFLX_TEST_CONFIG_JSON_H
