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

#ifndef WIFLX_COMMON_PACKET_H
#define WIFLX_COMMON_PACKET_H

#include <string>
#include <list>
#include <memory>
#include <common/clock.h>

namespace wiflx {
namespace common {

struct packet
{
  packet () :
    m_seqid (0)
  {}

  packet (std::string &&msdu, uint16_t seqid) :
    m_msdu (std::move (msdu)),
    m_seqid (seqid)
  {}

  packet (uint16_t seqid, std::shared_ptr<void> wpd) :
    m_seqid (seqid),
    m_wpd (wpd)
  {}

  packet (std::string &&msdu, uint16_t seqid, std::shared_ptr<void> wpd) :
    m_msdu (std::move (msdu)),
    m_seqid (seqid),
    m_wpd (wpd)
  {}

  // not copyable
  packet (const packet&) = delete;
  packet& operator= (const packet&) = delete;

  // movable
  packet (packet &&rhs) :
    m_msdu (std::move (rhs.m_msdu)),
    m_seqid (rhs.m_seqid),
    m_timestamp (std::move(rhs.m_timestamp)),
    m_wpd (std::move(rhs.m_wpd))
  {
    rhs.m_seqid = 0;
  }
  packet& operator= (packet &&rhs)
  {
    m_msdu = std::move (rhs.m_msdu);
    m_seqid = rhs.m_seqid;
    rhs.m_seqid = 0;
    m_timestamp = std::move(rhs.m_timestamp);
    m_wpd = std::move(rhs.m_wpd);
    return *this;
  }

  ~packet() = default;

  operator bool() const { return !m_msdu.empty (); }
  size_t size () const { return m_msdu.size (); }
  wiflx::common::clock::time_point timestamp () const { return m_timestamp; }

  std::string m_msdu;
  uint16_t m_seqid;
  wiflx::common::clock::time_point m_timestamp;
  // weak pointer to private data
  std::weak_ptr<void> m_wpd;
};
using packets = std::list<packet>;

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_PACKET_H
