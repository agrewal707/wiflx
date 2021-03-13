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

#include <common/fq_codel.h>

namespace wiflx {
namespace common {

fq_codel::fq_codel (const config::fq_codel &cfg, const config::codel &codel_cfg) :
  m_cfg (cfg),
  m_codel_cfg (codel_cfg),
  m_total_bytes (0),
  m_total_packets (0)
{}

void fq_codel::enqueue (const uint8_t tid, const uint32_t fid, const uint16_t nid, packet &&p)
{
  WIFLX_LOG_FUNCTION (this << unsigned(tid) << fid << nid);

  std::lock_guard<std::mutex> lock (m_mutex);
  auto it = m_tid_map.find (tid);
  if (it == m_tid_map.end ())
  {
    auto tctx = std::make_unique<tid_ctx> ();
    const auto result = m_tid_map.emplace (tid, std::move(tctx));
    if (!result.second)
      throw std::runtime_error ("duplicate tid");
    it = result.first;
    WIFLX_LOG_DEBUG ("NEW TID {:d}", tid);
  }
  auto &tctx = it->second;

  // max flows must be power of 2
  const auto tfid = fid & (m_cfg.max_flows-1);
  auto it2 = m_flow_map.find (tfid);
  if (it2 == m_flow_map.end ())
  {
    auto fctx = std::make_unique<flow_ctx> (tfid, nid, m_codel_cfg);
    const auto result = m_flow_map.emplace (tfid, std::move(fctx));
    if (!result.second)
      throw std::runtime_error ("duplicate fid");
    it2 = result.first;
    WIFLX_LOG_DEBUG ("NEW flow {:d}", tfid);
  }
  auto &flow = it2->second;

  if (flow->m_status == flow_ctx::INACTIVE)
  {
    flow->m_status = flow_ctx::NEW;
    flow->m_deficit = m_cfg.quantum;
    tctx->m_new_flows.push_back (flow.get());
  }

  const auto size = p.size ();
  if (flow->m_codel.enqueue (std::move(p)))
  {
    m_total_bytes += size;
    ++m_total_packets;
    WIFLX_LOG_DEBUG ("tctx {:d} flow {:d} deficit {:d} size {:d} total {:d}",
      unsigned(tid), flow->m_fid, flow->m_deficit, flow->m_codel.bytes (), m_total_packets);
  }

  if (m_total_packets > m_cfg.packet_limit)
    drop_packets ();
}

void fq_codel::drop_packets ()
{
  WIFLX_LOG_FUNCTION (this);

  // find queue with the largest current byte count
  uint32_t max_q_size = 0;
  flow_ctx *flow = nullptr;
  for (auto &[tid, tctx] : m_tid_map)
  {
    for (auto *val : tctx->m_new_flows)
    {
      auto q_size = val->m_codel.bytes ();
      if (q_size > max_q_size)
      {
        max_q_size = q_size;
        flow = val;
      }
    }
    for (auto *val : tctx->m_old_flows)
    {
      auto q_size = val->m_codel.bytes ();
      if (q_size > max_q_size)
      {
        max_q_size = q_size;
        flow = val;
      }
    }
  }

  if (!flow)
  {
    WIFLX_LOG_ERROR ("flow not found");
    return;
  }
  WIFLX_LOG_DEBUG ("found FAT flow {:d}, TXQ size {:d}", flow->m_fid, max_q_size);

  // drop half the number of packets (and bytes) from the head
  uint32_t size = 0, count = 0, threshold = max_q_size >> 1;
  do
  {
    auto [pkt, bytes_dropped, packets_dropped] = flow->m_codel.dequeue ();
    size += bytes_dropped;
    count += packets_dropped;
    m_total_bytes -= bytes_dropped;
    m_total_packets -= packets_dropped;
    if (pkt)
    {
      size += pkt.size ();
      ++count;
      m_total_bytes -= pkt.size ();
      --m_total_packets;
    }
  } while (count < m_cfg.batch_drop_size && size < threshold);

  WIFLX_LOG_DEBUG ("dropped {:d} packets from flow {:d}", count, flow->m_fid);
}

packets fq_codel::dequeue (const uint16_t nid)
{
  WIFLX_LOG_FUNCTION (this << nid);

  packets pkts;

  // TODO-loop through TIDs in priority order
  uint8_t tid = 0;

  std::lock_guard<std::mutex> lock (m_mutex);
  auto it = m_tid_map.find (tid);
  if (it == m_tid_map.end())
  {
    return pkts;
  }
  auto &tctx = it->second;

  flow_ctx *flow = nullptr;
  while (1)
  {
    bool found = false;
    std::list<flow_ctx*>::iterator it;

    for (it = tctx->m_new_flows.begin(); !found && it != tctx->m_new_flows.end();)
    {
      flow = *it;
      if (nid == flow->m_nid)
      {
        if (flow->m_deficit <= 0)
        {
          flow->m_deficit += m_cfg.quantum;
          flow->m_status = flow_ctx::OLD;
          tctx->m_old_flows.push_back (flow);
          it = tctx->m_new_flows.erase (it);
        }
        else
        {
          WIFLX_LOG_DEBUG ("NEW flow {:d} with positive flow deficit {:d}",
            flow->m_fid, flow->m_deficit);

          if (flow->m_codel.empty ())
          {
            WIFLX_LOG_DEBUG ("flow {:d} is EMPTY", flow->m_fid);
            flow->m_status = flow_ctx::OLD;
            tctx->m_old_flows.push_back (flow);
            it = tctx->m_new_flows.erase (it);
          }
          else
            found = true;
        }
      }
      else
        ++it;
    }

    for (it = tctx->m_old_flows.begin(); !found && it != tctx->m_old_flows.end();)
    {
      flow = *it;
      if (nid == flow->m_nid)
      {
        if (flow->m_deficit <= 0)
        {
          flow->m_deficit += m_cfg.quantum;
          tctx->m_old_flows.push_back (flow);
          it = tctx->m_old_flows.erase (it);
        }
        else
        {
          WIFLX_LOG_DEBUG ("OLD flow {:d} with positive flow deficit {:d}",
            flow->m_fid, flow->m_deficit);

          if (flow->m_codel.empty ())
          {
            WIFLX_LOG_DEBUG ("flow {:d} is EMPTY", flow->m_fid);
            flow->m_status = flow_ctx::INACTIVE;
            it = tctx->m_old_flows.erase (it);
          }
          else
            found = true;
        }
      }
      else
        ++it;
    }

    if (!found)
    {
      return pkts;
    }

    {
      size_t size = 0;
      do
      {
        auto [pkt, bytes_dropped, packets_dropped] = flow->m_codel.dequeue ();
        m_total_bytes -= bytes_dropped;
        m_total_packets -= packets_dropped;
        if (pkt)
        {
          flow->m_deficit -= pkt.size ();
          m_total_bytes -= pkt.size ();
          --m_total_packets;
          size += pkt.size ();
          pkts.push_back (std::move(pkt));
          WIFLX_LOG_DEBUG ("tid {:d} flow {:d} deficit {:d} bytes {:d} total bytes {:d} total packets {:d}",
            tid, flow->m_fid, flow->m_deficit, flow->m_codel.bytes (), m_total_bytes, m_total_packets);
        }
      } while (size < m_cfg.max_aggregate_size && !flow->m_codel.empty ());
      return pkts;
    }
  }
  return pkts;
}

} // namespace common
} // namespace wiflx
