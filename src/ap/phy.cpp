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

#include <ap/phy.h>

#include <common/log.h>
#include <common/profiling.h>

namespace wiflx {
namespace ap {

using std::mutex;
using std::unique_lock;
using std::lock_guard;

phy::phy (
  const common::config::radio &rcfg,
  const common::config::ofdm &ocfg) :
  m_rxbuff ("RX BUFFER", 16834),
  m_txbuff ("TX BUFFER", 16834),
  m_radio (rcfg, m_rxbuff, m_txbuff),
  m_ofdm_rx (ocfg, m_rxbuff, m_radio, this),
  m_ofdm_tx (ocfg, m_txbuff, m_radio),
  m_stopped (true)
{
  WIFLX_LOG_FUNCTION (this);

  m_radio.info (SOAPY_SDR_RX);
  m_radio.info (SOAPY_SDR_TX);
}

phy::~phy ()
{
  WIFLX_LOG_FUNCTION (this);
}

void phy::start ()
{
  WIFLX_LOG_FUNCTION (this);

  if (m_stopped)
  {
    m_stopped = false;
    m_radio.start ();
    m_ofdm_rx.start ();
    m_ofdm_tx.start ();
   }
}

void phy::stop ()
{
  WIFLX_LOG_FUNCTION (this);

  if (!m_stopped)
  {
    m_stopped = true;
    m_phy_q_available.notify_one ();
    m_radio.stop ();
    m_ofdm_rx.stop ();
    m_ofdm_tx.stop ();
  }
}

//
// RX
//
void phy::rx_run ()
{
  WIFLX_LOG_FUNCTION (this);

  WIFLX_PROFILING_SETTHREADNAME("RX THREAD");

  while (!m_stopped)
  {
    m_ofdm_rx.step ();
  }
}

void phy::on_receive (
  unsigned char *  _header,
  int              _header_valid,
  unsigned char *  _payload,
  unsigned int     _payload_len,
  int              _payload_valid,
  framesyncstats_s _stats)
{
  WIFLX_LOG_FUNCTION (this);

  mac_phy::sap_user::stats st;
  st.m_valid = _payload_valid;
  st.m_rssi = _stats.rssi;
  st.m_evm = _stats.evm;

  std::string mpdu (reinterpret_cast<const char*> (_payload), _payload_len);
  m_u->on_receive (std::move(mpdu), st);
}

//
// TX
//
void phy::send (std::string &&psdu)
{
  WIFLX_LOG_FUNCTION (this);

  {
    lock_guard<mutex> lock (m_phy_q_mutex);
    m_phy_q.push (std::move(psdu));
  }
  m_phy_q_available.notify_one ();
}

bool phy::wait_on_send (mac_phy::mcs &m, std::string &psdu)
{
  WIFLX_LOG_FUNCTION (this);

  unique_lock<mutex> lock (m_phy_q_mutex);
  m_phy_q_available.wait (lock, [this] () { return !m_phy_q.empty() || m_stopped; });
  if (!m_stopped)
  {
    psdu = std::move (m_phy_q.front ());
    m_phy_q.pop ();
    // TODO - MCS
  }

  return !m_stopped;
}

void phy::tx_run ()
{
  WIFLX_LOG_FUNCTION (this);

  WIFLX_PROFILING_SETTHREADNAME("TX THREAD");

  while (!m_stopped)
  {
    mac_phy::mcs m;
    std::string psdu;
    if (wait_on_send (m, psdu))
    {
      m_ofdm_tx.send (psdu);
      m_u->on_send (psdu.size ());
    }
  }
}

} // namespace ap
} // namespace wiflx
