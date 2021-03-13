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

#include <ap/l2iwf.h>

#include <common/log.h>
#include <common/error.h>
#include <common/net.h>
#include <common/hash.h>

namespace wiflx {
namespace ap {

const uint32_t l2iwf::s_max_buf_size = 2048; // bytes
const uint32_t l2iwf::s_stale_interval = 300; // seconds
const uint32_t l2iwf::s_stale_timeout = 300; // seconds

using common::eth_hdr;
using common::ip_hdr;
using common::tcp_hdr;
using common::udp_hdr;

l2iwf::l2iwf (const std::string &devname) :
  m_tuntap (m_ioc, devname),
  m_stale_timer (m_ioc)
{
  WIFLX_LOG_FUNCTION (this);
  m_read_buf.reserve (s_max_buf_size);
}

l2iwf::~l2iwf()
{
  WIFLX_LOG_FUNCTION (this);
}

void l2iwf::start ()
{
  WIFLX_LOG_FUNCTION (this);

  tuntap_receive ();
  stale_endpoint_check ();
}

void l2iwf::stop ()
{
  WIFLX_LOG_FUNCTION (this);

  m_tuntap.cancel ();
  m_stale_timer.cancel ();
}

void l2iwf::run ()
{
  WIFLX_LOG_FUNCTION (this);

  m_ioc.run ();
}

void l2iwf::tuntap_receive ()
{
  WIFLX_LOG_FUNCTION (this);

  m_read_buf.resize (s_max_buf_size);
  m_tuntap.async_receive (boost::asio::buffer(m_read_buf),
    [this] (const boost::system::error_code &ec, size_t bytes_transferred)
  {
    WIFLX_LOG_FUNCTION("receive" << ec << bytes_transferred);

    if (!ec)
    {
      m_read_buf.resize (bytes_transferred);
      uint16_t rid = common::messages::mac::BROADCAST_RID;
      auto *ethhdr = (eth_hdr *) m_read_buf.data();
      if (is_unicast(ethhdr->m_dmac))
      {
        const auto it = m_endpoint_map.find (ethhdr->m_dmac);
        if (it != m_endpoint_map.end ())
        {
          rid = it->second.m_rid;
        }
      }

      // Calculate flow hash
      uint8_t buf[25];
      memset (buf, 0, sizeof (buf));
      int i = 0;
      memcpy (buf, ethhdr->m_dmac.data(), 6);
      i += 6;
      memcpy (buf+i, ethhdr->m_smac.data(), 6);
      i += 6;

      const auto ethertype = ntohs (ethhdr->m_ethertype);
      if (ethertype == ETH_P_IP)
      {
        auto *iphdr = (ip_hdr*) ethhdr->m_data;

        const auto saddr = ntohl(iphdr->m_saddr);
        buf[i++] = (saddr >> 24) & 0xff;
        buf[i++] = (saddr >> 16) & 0xff;
        buf[i++] = (saddr >> 8) & 0xff;
        buf[i++] = (saddr >> 0) & 0xff;

        const auto daddr = ntohl(iphdr->m_daddr);
        buf[i++] = (daddr >> 24) & 0xff;
        buf[i++] = (daddr >> 16) & 0xff;
        buf[i++] = (daddr >> 8) & 0xff;
        buf[i++] = (daddr >> 0) & 0xff;

        buf[i++] = iphdr->m_proto;

        // TCP
        if (iphdr->m_proto == 6 && iphdr->m_frag_off == 0)
        {
          auto *tcphdr = (tcp_hdr*) iphdr->m_data;
          const auto sport = ntohl(tcphdr->m_sport);
          buf[i++] = (sport >> 8) & 0xff;
          buf[i++] = (sport >> 0) & 0xff;

          const auto dport = ntohl(tcphdr->m_dport);
          buf[i++] = (dport >> 8) & 0xff;
          buf[i++] = (dport >> 0) & 0xff;
        }
        else if (iphdr->m_proto == 17 && iphdr->m_frag_off == 0)
        {
          auto *udphdr = (udp_hdr*) iphdr->m_data;
          const auto sport = ntohl(udphdr->m_sport);
          buf[i++] = (sport >> 8) & 0xff;
          buf[i++] = (sport >> 0) & 0xff;

          const auto dport = ntohl(udphdr->m_dport);
          buf[i++] = (dport >> 8) & 0xff;
          buf[i++] = (dport >> 0) & 0xff;
        }
      }

      const auto fid = common::hashlittle (buf, sizeof (buf), 0);

      // TODO- map IPv4 TOS priority to tid
      m_mac->send (rid, 0, fid, std::move(m_read_buf));

      tuntap_receive ();
    }
    else if (ec != common::error::operation_aborted)
    {
      WIFLX_LOG_ERROR ("ec: {}", ec.message ());
      // TODO
    }
  });
}

void l2iwf::on_receive (const uint16_t rid, const std::string &ipdu)
{
  WIFLX_LOG_FUNCTION (rid << ipdu.size ());

  m_ioc.post ([=] () {
    const auto *hdr = (eth_hdr *) ipdu.data ();
    auto it = m_endpoint_map.find (hdr->m_smac);
    if (it == m_endpoint_map.end ())
    {
      it = m_endpoint_map.emplace(hdr->m_smac, ep_ctx (rid));
    }
    auto &ctx = it->second;

    ctx.m_expiry_time = common::clock::now () + std::chrono::seconds(s_stale_timeout);

    m_tuntap.async_send (boost::asio::buffer(ipdu),
      [this] (const boost::system::error_code &ec, size_t bytes_transferred)
      {
        WIFLX_LOG_FUNCTION("send:" << ec << bytes_transferred);
        if (ec && ec != common::error::operation_aborted)
        {
          WIFLX_LOG_ERROR ("ec: {}", ec.message());
        }
      });
  });
}

void l2iwf::stale_endpoint_check()
{
  WIFLX_LOG_FUNCTION (this);

  m_stale_timer.expires_from_now (
    std::chrono::seconds(s_stale_interval));
  m_stale_timer.async_wait(
    [this] (const boost::system::error_code &ec)
    {
      if (!ec)
      {
        stale_endpoint_check ();

        for (auto it = m_endpoint_map.begin(); it != m_endpoint_map.end ();)
        {
          const auto now = common::clock::now ();
          const auto diff = now - it->second.m_expiry_time;
          if (diff > std::chrono::seconds(s_stale_timeout))
            it = m_endpoint_map.erase (it);
          else
            ++it;
        }
      }
      else if (ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR ("ec: {}", ec.message());
      }
    });
}

} // namespace ap
} // namespace wiflx
