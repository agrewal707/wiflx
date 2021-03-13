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

#ifndef WIFLX_COMMON_FQ_CODEL_H
#define WIFLX_COMMON_FQ_CODEL_H

#include <list>
#include <mutex>
#include <common/log.h>
#include <common/config.h>
#include <common/codel.h>

namespace wiflx {
namespace common {

struct fq_codel
{
  //
  // flow context
  //
  struct flow_ctx
  {
    flow_ctx (const uint32_t fid, const uint16_t nid, const config::codel &codel_cfg) :
      m_fid (fid),
      m_nid (nid),
      m_deficit (0),
      m_status (INACTIVE),
      m_codel (codel_cfg)
    {}

    // not copyable
    flow_ctx (const flow_ctx&) = delete;
    flow_ctx& operator= (const flow_ctx&) = delete;

    // not movable
    flow_ctx (flow_ctx&&) = delete;
    flow_ctx& operator= (flow_ctx&&) = delete;

    ~flow_ctx () = default;

    enum status
    {
      INACTIVE,
      NEW,
      OLD
    };

    uint32_t m_fid;
    uint16_t m_nid;
    int32_t m_deficit;
    status m_status;
    codel m_codel;
  };

  struct tid_ctx
  {
    tid_ctx () = default;

    // not copyable
    tid_ctx (const tid_ctx&) = delete;
    tid_ctx& operator= (const tid_ctx&) = delete;

    // not movable
    tid_ctx (tid_ctx&&) = delete;
    tid_ctx& operator= (tid_ctx&&) = delete;

    ~tid_ctx () = default;

    std::list<flow_ctx*> m_new_flows;
    std::list<flow_ctx*> m_old_flows;
  };

  fq_codel (const config::fq_codel &cfg, const config::codel &codel_cfg);

  void enqueue (const uint8_t tid, const uint32_t fid, const uint16_t nid, packet &&p);
  void drop_packets();
  packets dequeue (const uint16_t nid);
  uint32_t get_total_buffered_bytes () const { return m_total_bytes; }

  const config::fq_codel &m_cfg;
  const config::codel &m_codel_cfg;
  // fid -> flow context
  using flow_map = std::unordered_map<uint32_t, std::unique_ptr<flow_ctx>>;
  flow_map m_flow_map;
  using tid_map = std::unordered_map<uint8_t, std::unique_ptr<tid_ctx>>;
  tid_map m_tid_map;
  std::mutex m_mutex;
  uint32_t m_total_bytes;
  uint32_t m_total_packets;
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_FQ_CODEL_H
