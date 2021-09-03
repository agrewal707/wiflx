/*
 * Copyright (c) 2012 Andrew McGregor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Codel, the COntrolled DELay Queueing discipline
 * Based on ns2 simulation code presented by Kathie Nichols
 *
 * This port based on linux kernel code by
 * Authors:	Dave Täht <d@taht.net>
 *		Eric Dumazet <edumazet@google.com>
 *
 * Ported to ns-3 by: Andrew McGregor <andrewmcgr@gmail.com>
 *
 * Modified for use in WiFLX by Ajay Pal S. Grewal <ajay.grewal.dev@gmail.com>
 */
#include <common/codel.h>
#include <common/log.h>

namespace wiflx {
namespace common {

codel::codel (const config::codel &cfg) :
  m_cfg (cfg),
  m_count (0),
  m_lastCount (0),
  m_dropping (false),
  m_recInvSqrt (~0U >> REC_INV_SQRT_SHIFT),
  m_firstAboveTime (0),
  m_dropNext (0)
{
  WIFLX_LOG_FUNCTION (this);
}

/*
 http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Iterative_methods_for_reciprocal_square_roots
 * new_invsqrt = (invsqrt / 2) * (3 - count * invsqrt^2)
 *
 * Here, invsqrt is a fixed point number (< 1.0), 32bit mantissa, aka Q0.32
 */
uint16_t codel::NewtonStep (uint16_t recInvSqrt, uint32_t count)
{
  WIFLX_LOG_FUNCTION(recInvSqrt << count);

  uint32_t invsqrt = ((uint32_t)recInvSqrt) << REC_INV_SQRT_SHIFT;
  uint32_t invsqrt2 = ((uint64_t)invsqrt * invsqrt) >> 32;
  uint64_t val = (3LL << 32) - ((uint64_t)count * invsqrt2);

  val >>= 2; /* avoid overflow in following multiply */
  val = (val * invsqrt) >> (32 - 2 + 1);
  return static_cast<uint16_t>(val >> REC_INV_SQRT_SHIFT);
}

uint32_t codel::ControlLaw (uint32_t t, uint32_t interval, uint32_t recInvSqrt)
{
  WIFLX_LOG_FUNCTION (t << interval << recInvSqrt);
  return t + ReciprocalDivide (interval, recInvSqrt << REC_INV_SQRT_SHIFT);
}

bool codel::enqueue (packet &&pkt)
{
  WIFLX_LOG_FUNCTION (this);

  if (m_txq.bytes ()+pkt.size () > m_cfg.max_size)
  {
    WIFLX_LOG_DEBUG ("Queue is full, dropping pkt");
    return false;
  }

  pkt.m_timestamp = clock::now ();
  WIFLX_LOG_DEBUG ("seq: {:d}", pkt.m_seqid);
  m_txq.push (std::move(pkt));
  WIFLX_LOG_DEBUG ("Number packets {:d}, bytes {:d}", m_txq.packets (), m_txq.bytes ());
  return true;
}

bool codel::OkToDrop (const packet &pkt, uint32_t now)
{
  WIFLX_LOG_FUNCTION (this);

  if (!pkt)
  {
    m_firstAboveTime = 0;
    return false;
  }

  const auto delta = wiflx::common::clock::now() - pkt.timestamp();
  WIFLX_LOG_INFO ("seq:{:d}, sojourn time:{:d}ms", pkt.m_seqid, std::chrono::duration_cast<std::chrono::milliseconds>(delta).count());
  const uint32_t sojournTime = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();

  if (CoDelTimeBefore (sojournTime, m_cfg.target) || m_txq.bytes () <= m_cfg.min_bytes)
  {
    // went below so we'll stay below for at least 'interval'
    WIFLX_LOG_DEBUG ("Sojourn time below target or number of bytes <= min bytes");
    m_firstAboveTime = 0;
    return false;
  }

  bool okToDrop = false;
  if (m_firstAboveTime == 0)
  {
    /* just went above from below. If we stay above
     * for at least 'interval' we'll say it's ok to drop
     */
    WIFLX_LOG_DEBUG ("Sojourn time has gone above target from below");
    m_firstAboveTime = now + m_cfg.interval;
  }
  else if (CoDelTimeAfter (now, m_firstAboveTime))
  {
    WIFLX_LOG_DEBUG ("Sojourn time has been above target for at least 'interval', it's OK to (possibly) drop packet.");
    okToDrop = true;
  }
  return okToDrop;
}

std::tuple<packet,uint32_t,uint32_t> codel::dequeue (void)
{
  WIFLX_LOG_FUNCTION (this);

  std::tuple<packet,uint32_t,uint32_t> retval {packet(), 0, 0};
  auto& [pkt, bytes_dropped, packets_dropped] = retval;

  pkt = m_txq.pop ();
  if (!pkt)
  {
    // Leave dropping state when queue is empty
    m_dropping = false;
    WIFLX_LOG_DEBUG ("Queue empty");
    return retval;
  }

  WIFLX_LOG_DEBUG ("Number packets {:d}, bytes {:d}", m_txq.packets (), m_txq.bytes ());

  uint32_t now = CoDelGetTime ();

  // Determine if item should be dropped
  bool okToDrop = OkToDrop (pkt, now);

  if (m_dropping)
  { // In the dropping state (sojourn time has gone above target and hasn't come down yet)
    // Check if we can leave the dropping state or next drop should occur
    WIFLX_LOG_DEBUG ("In dropping state, check if it's OK to leave or next drop should occur");
    if (!okToDrop)
    {
      /* sojourn time fell below target - leave dropping state */
      WIFLX_LOG_DEBUG ("Sojourn time below target, leave dropping state.");
      m_dropping = false;
    }
    else if (CoDelTimeAfterEq (now, m_dropNext))
    {
      // It's time for the next drop. Drop the current packet and
      // dequeue the next. The dequeue might take us out of dropping
      // state. If not, schedule the next drop.
      // A large amount of packets in queue might result in drop
      // rates so high that the next drop should happen now,
      // hence the while loop.
      while (m_dropping && CoDelTimeAfterEq (now, m_dropNext))
      {
        ++m_count;
        m_recInvSqrt = NewtonStep (m_recInvSqrt, m_count);

        WIFLX_LOG_WARNING ("Sojourn time is still above target and it's time for next drop, drop seq:{:d}", pkt.m_seqid);
        bytes_dropped += pkt.size();
        ++packets_dropped;

        pkt = m_txq.pop ();
        if (pkt)
        {
          WIFLX_LOG_DEBUG ("Popped, Number remaining packets {:d}, bytes {:d}", m_txq.packets (), m_txq.bytes ());
        }

        if (!OkToDrop (pkt, now))
        {
          /* leave dropping state */
          WIFLX_LOG_DEBUG ("Leaving dropping state");
          m_dropping = false;
        }
        else
        {
          /* schedule the next drop */
          WIFLX_LOG_DEBUG ("Running ControlLaw for input m_dropNext: {:f}", (double)m_dropNext / 1000000);
          m_dropNext = ControlLaw (m_dropNext, m_cfg.interval, m_recInvSqrt);
          WIFLX_LOG_DEBUG ("Scheduled next drop at {:f}", (double)m_dropNext / 1000000);
        }
      }
    }
  }
  else
  {
    // Not in the dropping state
    // Decide if we have to enter the dropping state and drop the first packet
    WIFLX_LOG_DEBUG ("Not in dropping state; decide if we have to enter the state and drop the first packet");
    if (okToDrop)
    {
      // Drop the first packet and enter dropping state unless the queue is empty
      WIFLX_LOG_WARNING ("Sojourn time above target, enter dropping state, drop seq:{:d}", pkt.m_seqid);
      bytes_dropped += pkt.size();
      ++packets_dropped;

      pkt = m_txq.pop();
      if (pkt)
      {
        WIFLX_LOG_DEBUG ("Popped, Number packets {:d}, bytes {:d}", m_txq.packets (), m_txq.bytes ());
      }

      OkToDrop (pkt, now);
      m_dropping = true;

      /*
       * if min went above target close to when we last went below it
       * assume that the drop rate that controlled the queue on the
       * last cycle is a good starting point to control it now. The 'm_dropNext'
       * will be at most 'INTERVAL' later than the time of the last drop, so
       * 'now - m_dropNext' is a good approximation of the time from the last drop
       * until now.
       */
      int delta = m_count - m_lastCount;
      if (delta > 1 && CoDelTimeBefore (now - m_dropNext, 16 * m_cfg.interval))
      {
        m_count = delta;
        m_recInvSqrt = NewtonStep (m_recInvSqrt, m_count);
      }
      else
      {
        m_count = 1;
        m_recInvSqrt = ~0U >> REC_INV_SQRT_SHIFT;
      }
      m_lastCount = m_count;
      WIFLX_LOG_DEBUG ("Running ControlLaw for input now: {:f}", (double)now);
      m_dropNext = ControlLaw (now, m_cfg.interval, m_recInvSqrt);
      WIFLX_LOG_DEBUG ("Scheduled next drop at {:f}, now {:f}", (double)m_dropNext / 1000000, (double)now / 1000000);
    }
  }

  WIFLX_LOG_DEBUG ("bytes_dropped {:d}, packets dropped {:d}", bytes_dropped, packets_dropped);
  return retval;
}

} // namespace common
} // namespace wiflx
