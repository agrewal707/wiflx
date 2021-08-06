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
#ifndef WIFLX_TEST_OFDM_PHY_TEST_H
#define WIFLX_TEST_OFDM_PHY_TEST_H

#include <test/test_base.h>
#include <test/ofdm_phy.h>
#include <test/mac_phy_interface.h>

namespace wiflx {
namespace test {

struct ofdm_phy_test :
  public test_base,
  public mac_phy::sap_user
{
  ofdm_phy_test (const wiflx::test::config &cfg) :
    test_base (cfg),
    m_cfg (cfg),
    m_phy (cfg.m_radio, cfg.m_ofdm.ofdm),
    m_tx_seq (0),
    m_rx_seq (0),
    m_rx_total (0),
    m_rx_ok (0),
    m_rx_ko (0),
    m_rx_dropped (0),
    m_rx_ooseq (0)
  {
    WIFLX_LOG_FUNCTION (this);

    m_send_psdu.resize(cfg.m_ofdm.sdu_size);
    char *p = m_send_psdu.data();
    for (int i = 4; i < m_send_psdu.size (); ++i)
    {
      p[i] = i & 0xff;
    }

    m_phy.set_mac_phy_sap_user (this);
  }

  ~ofdm_phy_test ()
  {
    WIFLX_LOG_FUNCTION (this);

    WIFLX_LOG_WARNING ("TX         {:d}", m_tx_seq);
    WIFLX_LOG_WARNING ("RX TOTAL   {:d}", m_rx_total);
    WIFLX_LOG_WARNING ("RX OK      {:d}", m_rx_ok);
    WIFLX_LOG_WARNING ("RX KO      {:d}", m_rx_ko);
    WIFLX_LOG_WARNING ("RX DROPPED {:d}", m_rx_dropped);
    WIFLX_LOG_WARNING ("RX OOSEQ   {:d}", m_rx_ooseq);
  }

  virtual void start()
  {
    WIFLX_LOG_FUNCTION (this);

    m_phy.start ();

    #ifdef WIFLX_RT_SCHED
      m_phy_rx_thread = std::move (std::thread (&wiflx::test::ofdm_phy::rx_run, &m_phy));
      //wiflx::common::set_thread_param (m_phy_rx_thread.native_handle(), "PHY RX", SCHED_FIFO, 1, 0);
      wiflx::common::set_thread_param (m_phy_rx_thread.native_handle(), "PHY RX", -1, 0, -1);

      m_phy_tx_thread = std::move (std::thread (&wiflx::test::ofdm_phy::tx_run, &m_phy));
      wiflx::common::set_thread_param (m_phy_tx_thread.native_handle(), "PHY TX", SCHED_FIFO, 2, 0);
    #else
      m_rx_thread = std::move (std::thread (&wiflx::test::ofdm_phy::rx_run, &p));
      m_tx_thread = std::move (std::thread (&wiflx::test::ofdm_phy::tx_run, &p));
    #endif

		if (m_cfg.m_ofdm.tx_enable)
			send ();
  }

  virtual void stop ()
  {
    m_phy.stop ();
    m_phy_rx_thread.join();
    m_phy_tx_thread.join();
  }

	void send ()
	{
    auto psdu = m_send_psdu;
    char *p = psdu.data();
    *p++ = m_tx_seq >> 24;
    *p++ = m_tx_seq >> 16;
    *p++ = m_tx_seq >> 8;
    *p++ = m_tx_seq >> 0;
		m_phy.send (std::move(psdu));
    m_tx_seq++;
	}

  virtual void on_send ()
  {
    send ();
  }

  virtual void on_receive (std::string &&psdu, const stats &st)
  {
    m_rx_total++;

    common::log_hexdump ((const unsigned char*)psdu.data(), psdu.size(), 16);

    if (st.m_valid)
    {
      m_rx_ok++;
      char *p = psdu.data();
      uint32_t rx_seq = 0;
      rx_seq = *p++ << 24;
      rx_seq |= *p++ << 16;
      rx_seq |= *p++ << 8;
      rx_seq |= *p++ << 0;

      auto dropped = rx_seq - m_rx_seq - 1;
      if (dropped > 0)
      {
        m_rx_dropped += dropped;
        WIFLX_LOG_ERROR ("{:d} DROPPED", dropped);
      }
      else
      {
        m_rx_ooseq++;
        WIFLX_LOG_ERROR ("OUT OF SEQ");
      }

      m_rx_seq = rx_seq;
    }
    else
    {
      m_rx_ko++;
      WIFLX_LOG_ERROR ("PACKET INVALID");
    }
  }

  config m_cfg;
  ofdm_phy m_phy;
  std::thread m_phy_rx_thread;
  std::thread m_phy_tx_thread;
	std::string m_send_psdu;
  uint32_t m_tx_seq;
  uint32_t m_rx_seq;
  uint32_t m_rx_total;
  uint32_t m_rx_ok;
  uint32_t m_rx_ko;
  uint32_t m_rx_ooseq; // out of sequence
  uint32_t m_rx_dropped;
};
WIFLX_OBJECT_FACTORY_REGISTER_IMPL(test_base,ofdm_phy_test);

} // namespace test
} // namespace wiflx

#endif // WIFLX_TEST_OFDM_PHY_TEST_H
