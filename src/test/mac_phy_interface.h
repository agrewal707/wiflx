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

#ifndef WIFLX_TEST_MAC_PHY_INTERFACE_H
#define WIFLX_TEST_MAC_PHY_INTERFACE_H

namespace wiflx {
namespace test {
namespace mac_phy {

enum mcs
{
  MCS_1 = 1
};

struct sap_user
{
  struct stats
  {
    stats () :
      m_valid (false),
      m_rssi (0.0f),
      m_evm (0.0f)
    {}

    bool m_valid;
    float m_rssi;
    float m_evm;
  };
  virtual ~sap_user () {};

  virtual void on_receive (std::string &&mpdu, const stats &st) = 0;
  virtual void on_send () = 0;
};

struct sap_provider
{
  virtual ~sap_provider () {};
  virtual void send (std::string &&psdu) = 0;
};

} // namespace mac_phy
} // namespace test
} // namespace wiflx

#endif // WIFLX_TEST_MAC_PHY_INTERFACE_H
