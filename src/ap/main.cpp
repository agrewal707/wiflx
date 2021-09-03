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
#include <thread>
#include <config.h>
#include <common/log.h>
#include <common/gpio.h>
#include <common/utils.h>
#include <common/os_utils.h>
#include <common/io_context.h>
#include <common/profiling.h>
#include <ap/config_json.h>
#include <ap/phy.h>
#include <ap/mac.h>
#include <ap/l2iwf.h>

int main (int C, char *V[])
{
  using json = nlohmann::json;

  if (C < 2)
  {
    fprintf(stderr, "Usage: %s ap.cfg [logfile]\n", V[0]);
    exit(-1);
  }

  try
	{
    if (C < 3)
      wiflx::common::log_init ();
    else
      wiflx::common::log_init (std::string(V[2]));

    wiflx::common::log_set_level (quill::LogLevel::Debug);

    WIFLX_GPIO_INIT();
    WIFLX_PROFILING_SETTHREADNAME("MAIN THREAD");

    WIFLX_LOG_DEBUG ("WIFLX STARTED");

    const auto cfgstr = wiflx::common::get_file_content (V[1]);
    const json j = json::parse (cfgstr, nullptr, true, true);
    const auto cfg = j.get<wiflx::ap::config> ();

    // PHY
    wiflx::ap::phy p (cfg.m_radio, cfg.m_ofdm);

    // MAC
    wiflx::ap::mac m (cfg.m_mac, cfg.m_fq_codel, cfg.m_codel);

    // L2/ETH
    wiflx::ap::l2iwf l2 (cfg.m_l2iwf.devname);

    // IWF - MAC
    l2.set_iwf_mac_sap_provider (&m);
    m.set_iwf_mac_sap_user (&l2);

    // MAC-PHY
    m.set_mac_phy_sap_provider (&p);
    p.set_mac_phy_sap_user (&m);

    p.start ();
    m.start ();
    l2.start ();

    auto phy_rx_thread = std::move (std::thread (&wiflx::ap::phy::rx_run, &p));
    auto phy_tx_thread = std::move (std::thread (&wiflx::ap::phy::tx_run, &p));
    auto mac_timer_thread = std::move (std::thread (&wiflx::ap::mac::timer_run, &m));
    auto l2iwf_thread = std::move (std::thread (&wiflx::ap::l2iwf::run, &l2));

#ifdef WIFLX_RT_SCHED
    wiflx::common::set_thread_param (phy_rx_thread.native_handle(), "PHY RX", SCHED_FIFO, 1, 0);
    wiflx::common::set_thread_param (phy_tx_thread.native_handle(), "PHY TX", SCHED_FIFO, 2, 0);
    wiflx::common::set_thread_param (mac_timer_thread.native_handle(), "MAC TIMER", -1, 0, -1);
    wiflx::common::set_thread_param (l2iwf_thread.native_handle(), "L2 NETWORK", -1, 0, -1);
#else

#endif

    wiflx::common::io_context ioc;
    boost::asio::signal_set signals (ioc, SIGINT, SIGTERM);
    signals.async_wait (
      [&] (const boost::system::error_code &ec, int signal)
        {
          l2.stop ();
          m.stop ();
          p.stop ();
          ioc.stop ();
        });

    ioc.run ();

    phy_rx_thread.join();
    phy_tx_thread.join();
    mac_timer_thread.join ();
    l2iwf_thread.join ();
  }
  catch (const std::exception &ex)
  {
    WIFLX_LOG_ERROR ("{}", ex.what());
  }

  return 0;
}
