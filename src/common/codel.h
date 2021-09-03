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
#ifndef WIFLX_COMMON_CODEL_H
#define WIFLX_COMMON_CODEL_H

#include <common/config.h>
#include <common/clock.h>
#include <common/log.h>
#include <common/txq.h>
#include <common/packet.h>

namespace wiflx {
namespace common {

 /* CoDel uses a 1024 nsec clock, encoded in u32
  * This gives a range of 2199 seconds, because of signed compares
  */
#define CODEL_SHIFT 10
#define REC_INV_SQRT_BITS (8 * sizeof(uint16_t)) /* or sizeof_in_bits(recInvSqrt) */
/* needed shift to get a Q0.32 number from recInvSqrt */
#define REC_INV_SQRT_SHIFT (32 - REC_INV_SQRT_BITS)

class codel
{
public:
  codel (const config::codel &cfg);

  bool enqueue (packet &&p);
  std::tuple<packet,uint32_t,uint32_t> dequeue ();
  packet& front () { return m_txq.front(); }
  bool empty () const { return m_txq.empty (); }
  uint32_t bytes () const { return m_txq.bytes (); }
  uint32_t packets () const { return m_txq.packets (); }

private:
  /**
   * \brief Calculate the reciprocal square root of m_count by using Newton's method
   * \param recInvSqrt reciprocal value of sqrt (count)
   * \param count count value
   * \return The new recInvSqrt value
   */
  static uint16_t NewtonStep (uint16_t recInvSqrt, uint32_t count);

  /**
   * \brief Determine the time for next drop
   * CoDel control law is t + m_interval/sqrt(m_count).
   * Here, we use m_recInvSqrt calculated by Newton's method in NewtonStep() to avoid
   * both sqrt() and divide operations
   *
   * \param t Current next drop time (in units of CoDel time)
   * \param interval interval (in units of CoDel time)
   * \param recInvSqrt reciprocal value of sqrt (count)
   * \return The new next drop time (in units of CoDel time)
   */
  static uint32_t ControlLaw (uint32_t t, uint32_t interval, uint32_t recInvSqrt);

  /**
   * \brief Determine whether a packet is OK to be dropped. The packet
   * may not be actually dropped (depending on the drop state)
   *
   * \param item The packet that is considered
   * \param now The current time represented as 32-bit unsigned integer (us)
   * \returns True if it is OK to drop the packet (sojourn time above target for at least interval)
   */
  bool OkToDrop (const packet &p, uint32_t now);

  /**
   * Check if CoDel time a is successive to b
   * @param a left operand
   * @param b right operand
   * @return true if a is greater than b
   */
  static bool CoDelTimeAfter (uint32_t a, uint32_t b)
  {
    return ((int64_t)(a) - (int64_t)(b) > 0);
  }
  /**
   * Check if CoDel time a is successive or equal to b
   * @param a left operand
   * @param b right operand
   * @return true if a is greater than or equal to b
   */
  static bool CoDelTimeAfterEq (uint32_t a, uint32_t b)
  {
    return ((int64_t)(a) - (int64_t)(b) >= 0);
  }
  /**
   * Check if CoDel time a is preceding b
   * @param a left operand
   * @param b right operand
   * @return true if a is less than to b
   */
  static bool CoDelTimeBefore (uint32_t a, uint32_t b)
  {
    return  ((int64_t)(a) - (int64_t)(b) < 0);
  }
  /**
   * Check if CoDel time a is preceding or equal to b
   * @param a left operand
   * @param b right operand
   * @return true if a is less than or equal to b
   */
  static bool CoDelTimeBeforeEq (uint32_t a, uint32_t b)
  {
    return ((int64_t)(a) - (int64_t)(b) <= 0);
  }

  /**
   * Performs a reciprocal divide, similar to the
   * Linux kernel reciprocal_divide function
   * \param A numerator
   * \param R reciprocal of the denominator B
   * \return the value of A/B
   */
  /* borrowed from the linux kernel */
  static uint32_t ReciprocalDivide (uint32_t A, uint32_t R)
  {
    return (uint32_t)(((uint64_t)A * R) >> 32);
  }
  /* end kernel borrowings */

  /**
   * Returns the current time translated in CoDel time representation
   * \return the current time
   */
  static uint32_t CoDelGetTime (void)
  {
    const auto now = wiflx::common::clock::now ();
    const uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds> (now.time_since_epoch()).count();
    return ns >> CODEL_SHIFT;
  }

  const config::codel &m_cfg;
  wiflx::common::txq_ctx m_txq;
  uint32_t m_count;          //!< Number of packets dropped since entering drop state
  uint32_t m_lastCount;      //!< Last number of packets dropped since entering drop state
  bool m_dropping;           //!< True if in dropping state
  uint16_t m_recInvSqrt;     //!< Reciprocal inverse square root
  uint32_t m_firstAboveTime; //!< Time to declare sojourn time above target
  uint32_t m_dropNext;       //!< Time to drop next packet
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_CODEL_H
