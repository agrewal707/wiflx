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

#include <rm/l2iwf.h>

#include <common/log.h>
#include <common/error.h>
#include <common/net.h>
#include <common/hash.h>

namespace wiflx {
namespace rm {

const uint32_t l2iwf::s_max_buf_size = 2048; // bytes

using common::eth_hdr;
using common::ip_hdr;
using common::tcp_hdr;
using common::udp_hdr;

l2iwf::l2iwf (const std::string &devname) :
  m_tuntap (m_ioc, devname)
{
  WIFLX_LOG_FUNCTION(this);
  m_read_buf.reserve (s_max_buf_size);
}

l2iwf::~l2iwf()
{
  WIFLX_LOG_FUNCTION(this);
}

void l2iwf::start ()
{
  WIFLX_LOG_FUNCTION (this);

  tuntap_receive ();
}

void l2iwf::stop ()
{
  WIFLX_LOG_FUNCTION (this);

  m_tuntap.cancel ();
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
    WIFLX_LOG_FUNCTION(ec << bytes_transferred);

    if (!ec)
    {
      m_read_buf.resize (bytes_transferred);
      auto *ethhdr = (eth_hdr *) m_read_buf.data();

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
      m_mac->send (0, fid, std::move(m_read_buf));

      tuntap_receive ();
    }
    else if (ec != common::error::operation_aborted)
    {
      WIFLX_LOG_ERROR ("ec: {}", ec.message ());
      // TODO
    }
  });
}

void l2iwf::on_receive (const std::string &ipdu)
{
  WIFLX_LOG_FUNCTION (ipdu.size ());

  m_tuntap.async_send (boost::asio::buffer(ipdu),
    [this] (const boost::system::error_code &ec, size_t bytes_transferred)
    {
      WIFLX_LOG_FUNCTION(ec << bytes_transferred);
      if (ec && ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR ("ec: {}", ec.message());
      }
    });
}

} // namespace rm
} // namespace wiflx
