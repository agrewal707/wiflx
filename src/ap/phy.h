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

#ifndef WIFLX_AP_PHY_H
#define WIFLX_AP_PHY_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <common/io_context.h>
#include <common/ofdm.h>
#include <common/radio.h>
#include <ap/mac_phy_interface.h>
#include <ap/config.h>

namespace wiflx {
namespace ap {

class phy :
  public mac_phy::sap_provider,
  public common::ofdm_rx::listener
{
public:
  phy (const common::config::radio &rcfg,
       const common::config::ofdm &ocfg);
  ~phy ();

  void set_mac_phy_sap_user (mac_phy::sap_user *u)
  {
    m_u = u;
  }

  void start ();
  void stop ();
  void rx_run ();
  void tx_run ();
  virtual void send (std::string &&psdu);

private:
  //
  // RX
  //
  virtual void on_receive (
    unsigned char      *_header,
    int                _header_valid,
    unsigned char     *_payload,
    unsigned int       _payload_len,
    int                _payload_valid,
    framesyncstats_s   _stats);

  //
  // TX
  //
  bool wait_on_send (mac_phy::mcs &m, std::string &psdu);

  mac_phy::sap_user *m_u;
  wiflx::common::pipebuf_cf m_rxbuff;
  wiflx::common::pipebuf_cf m_txbuff;
  wiflx::common::radio m_radio;
  wiflx::common::ofdm_tx m_ofdm_tx;
  wiflx::common::ofdm_rx m_ofdm_rx;
  std::queue<std::string> m_phy_q;
  std::mutex m_phy_q_mutex;
  std::condition_variable m_phy_q_available;
  bool m_stopped;
};

} // namespace ap
} // namespace wiflx

#endif // WIFLX_AP_PHY_H
