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

#ifndef WIFLX_COMMON_TXQ_H
#define WIFLX_COMMON_TXQ_H

#include <list>
#include <common/packet.h>

namespace wiflx {
namespace common {

struct txq_ctx
{
  txq_ctx () :
    m_bytes (0),
    m_packets (0)
  {}

  // not copyable
  txq_ctx (const txq_ctx&) = delete;
  txq_ctx& operator= (const txq_ctx&) = delete;

  // not movable
  txq_ctx (txq_ctx&&) = delete;
  txq_ctx& operator= (txq_ctx&&) = delete;

  packet& front ()
  {
    return m_q.front ();
  }

  void push (packet &&p)
  {
    const auto size = p.size ();
    m_q.push_back (std::move (p));
    m_bytes += size;
    ++m_packets;
  }

  packet pop ()
  {
    if (!empty())
    {
      auto pkt = std::move(m_q.front ());
      m_q.pop_front ();
      m_bytes -= pkt.size ();
      --m_packets;
      return pkt;
    }
    return packet ();
  }

  void clear ()
  {
    m_q.clear ();
  }

  uint32_t bytes () const
  {
    return m_bytes;
  }

  uint32_t packets () const
  {
    return m_packets;
  }

  bool empty () const
  {
    return m_q.empty ();
  }

  std::list<packet> m_q;
  uint32_t m_bytes;
  uint32_t m_packets;
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_TXQ_H
