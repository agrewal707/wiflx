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

#include <ap/mac.h>

#include <common/log.h>
#include <common/error.h>

namespace wiflx {
namespace ap {

using std::mutex;
using std::unique_lock;
using std::lock_guard;
using std::shared_ptr;
using std::weak_ptr;
using std::pair;

void mac::schedule ()
{
  WIFLX_LOG_FUNCTION (this);

  auto [ wrm, plp ] = get_next_remote ();

  send_u_ack_pdu (wrm);

  send_pdu (wrm, plp);
}

void mac::schedule_retry ()
{
  WIFLX_LOG_FUNCTION (this);

  m_ack_timer.expires_from_now (
    std::chrono::milliseconds (s_general_failure_timeout));
  m_ack_timer.async_wait (
    [this] (const boost::system::error_code &ec)
    {
      if (!ec)
      {
        schedule ();
      }
      else if (ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR ("ec: {}", ec.message ());
      }
    });
}

pair<weak_ptr<mac::rm_ctx>, mac::pktsptr> mac::get_next_remote ()
{
  pair<weak_ptr<rm_ctx>, pktsptr> retval;
  auto &wrm = retval.first;
  retval.second = std::make_shared<packets> ();
  auto &pkts = *(retval.second);

  {
    lock_guard<mutex> lock (m_txq_mutex);
    if (!m_retry_txq.empty ())
    {
      auto pkt = std::move (m_retry_txq.front ());
      m_retry_txq.pop ();
      pkts.push_back(std::move (pkt));
    }
    else if (!m_release_txq.empty ())
    {
      auto pkt = std::move (m_release_txq.front ());
      m_release_txq.pop ();
      pkts.push_back(std::move (pkt));
    }
  }

  if (!pkts.empty())
  {
    auto &pkt = pkts.front();
    if (auto pd = pkt.m_wpd.lock ())
    {
      wrm = std::static_pointer_cast<rm_ctx> (pd);
    }
    return retval;
  }

  if (m_b_poll_airtime_deficit < 0)
  {
    m_b_poll_airtime_deficit = m_cfg.b_poll_airtime_quantum;
    return retval;
  }

  {
    bool found = false;
    {
      lock_guard<mutex> lock (m_rm_list_mutex);
      while (!found && !m_new_rms.empty ())
      {
        wrm = m_new_rms.front ();
        if (auto rm = wrm.lock())
        {
          lock_guard<mutex> lock2 (rm->m_mutex);
          if (rm->m_deficit <= 0)
          {
            rm->m_deficit += m_cfg.rm_airtime_quantum;
            m_old_rms.push_back (wrm);
            m_new_rms.pop_front ();
          }
          else
          {
            WIFLX_LOG_DEBUG ("NEW rm {:d} with positive rm deficit {:d}",
              rm->m_rid, rm->m_deficit);
            found = true;
          }
        }
      }
      while (!found && !m_old_rms.empty ())
      {
        wrm = m_old_rms.front ();
        if (auto rm = wrm.lock())
        {
          lock_guard<mutex> lock2 (rm->m_mutex);
          if (rm->m_deficit <= 0)
          {
            rm->m_deficit += m_cfg.rm_airtime_quantum;
            m_old_rms.push_back (wrm);
            m_old_rms.pop_front ();
          }
          else
          {
            WIFLX_LOG_DEBUG ("OLD rm {:d} with positive rm deficit {:d}",
              rm->m_rid, rm->m_deficit);
            found = true;
          }
        }
      }
    }

    if (!found)
    {
      WIFLX_LOG_DEBUG ("No active remote found");
      wrm.reset();
      return retval;
    }

    if (auto rm = wrm.lock())
    {
      pkts = m_fq_codel.dequeue (rm->m_rid);
      if (pkts.empty())
      {
        WIFLX_LOG_DEBUG ("rm {:d} queues are EMPTY", rm->m_rid);
        {
          lock_guard<mutex> lock (m_rm_list_mutex);
          if (!m_new_rms.empty ())
          {
            WIFLX_LOG_DEBUG ("move rm {:d} to OLD", rm->m_rid);
            m_old_rms.push_back (wrm);
            m_new_rms.pop_front ();
          }
          else
          {
            if (is_bcast_rid(rm->m_rid))
            {
              // Only deactivate broadcast remote if is in OLD list
              // and has no packets. Since for other remotes
              // we'd like to send poll for uplink data.
              WIFLX_LOG_DEBUG ("deactivate rm {:d}", rm->m_rid);
              {
                lock_guard<mutex> lock2 (rm->m_mutex);
                rm->m_active = false;
              }
            }
            else
            {
              m_old_rms.push_back (wrm);
            }
            m_old_rms.pop_front ();
          }
        }

        // deactivate remote if inactivity timeout = 0 and
        // UL BSR = 0 (i.e. there is no pending uplink data)
        if (!m_cfg.inactivity_timeout)
        {
          unique_lock<mutex> lock2 (rm->m_mutex);
          if (!rm->m_bsr)
          {
            pkts.push_back (packet (++rm->m_seqid, rm));
            lock2.unlock ();
            remove_from_active_list (rm);
            WIFLX_LOG_INFO ("BSR NULL -- rid:{:d}", rm->m_rid);
          }
        }
      }
    }
  }

  return retval;
}

} // namespace ap
} // namespace wiflx
