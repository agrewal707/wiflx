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
#ifndef WIFLX_COMMON_NET_H
#define WIFLX_COMMON_NET_H

#include <linux/if_ether.h>
#include <asm/byteorder.h>

namespace wiflx {
namespace common {

struct eth_hdr
{
  std::array<uint8_t,6> m_dmac;
  std::array<uint8_t,6> m_smac;
  uint16_t m_ethertype;
  uint8_t  m_data[];
} __attribute__((packed));

struct ip_hdr
{
#if defined(__LITTLE_ENDIAN_BITFIELD)
	uint8_t	m_ihl:4,
	        m_version:4;
#elif defined (__BIG_ENDIAN_BITFIELD)
	uint8_t m_version:4,
  		    m_ihl:4;
#else
#error	"Please fix <asm/byteorder.h>"
#endif
    uint8_t m_tos;
    uint16_t m_tot_len;
    uint16_t m_id;
    uint16_t m_frag_off;
    uint8_t m_ttl;
    uint8_t m_proto;
    uint16_t m_check;
    uint32_t m_saddr;
    uint32_t m_daddr;
    uint8_t m_data[];
} __attribute__((packed));

struct tcp_hdr {
    uint16_t m_sport;
    uint16_t m_dport;
    uint8_t m_data[];
} __attribute__((packed));

struct udp_hdr {
    uint16_t m_sport;
    uint16_t m_dport;
    uint8_t m_data[];
} __attribute__((packed));

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_NET_H
