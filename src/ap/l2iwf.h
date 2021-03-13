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

#ifndef WIFLX_AP_L2IWF_H
#define WIFLX_AP_L2IWF_H

#include <unordered_map>
#include <common/io_context.h>
#include <common/clock.h>
#include <common/timer.h>
#include <common/utils.h>
#include <common/tuntap.h>
#include <ap/iwf_mac_interface.h>

namespace wiflx {
namespace ap {

class l2iwf :
  public iwf_mac::sap_user
{
public:
  l2iwf (const std::string &devname);
  ~l2iwf ();

  void set_iwf_mac_sap_provider (iwf_mac::sap_provider *m)
  {
    m_mac = m;
  }

  void start ();
  void stop ();
  void run ();
  virtual void on_receive (const uint16_t rid, const std::string &ipdu);

private:
  struct ep_ctx
  {
    ep_ctx (const uint16_t rid):
      m_rid (rid)
    {}
    uint16_t m_rid;
    common::clock::time_point m_expiry_time;
  };

  using mac_address = std::array<uint8_t,6>;

  void tuntap_receive ();
  void stale_endpoint_check ();

  wiflx::common::io_context m_ioc;
  iwf_mac::sap_provider *m_mac;
  common::tuntap m_tuntap;
  std::string m_read_buf;
  using endpoint_map = std::unordered_multimap<mac_address, ep_ctx>;
  endpoint_map m_endpoint_map;
  wiflx::common::timer m_stale_timer;

  static bool is_unicast (const mac_address &addr)
  {
    return !(addr[0] & 0x01);
  }

  static const uint32_t s_max_buf_size;
  static const uint32_t s_stale_interval;
  static const uint32_t s_stale_timeout;
};

} // namespace ap
} // namespace wiflx

#endif // WIFLX_AP_L2IWF_H
