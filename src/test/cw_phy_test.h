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
#ifndef WIFLX_TEST_CW_PHY_TEST_H
#define WIFLX_TEST_CW_PHY_TEST_H

#include <config.h>
#include <common/os_utils.h>
#include <test/test_base.h>
#include <test/cw_phy.h>

namespace wiflx {
namespace test {

struct cw_phy_test :
  public test_base
{
  cw_phy_test (const wiflx::test::config &cfg) :
    test_base (cfg),
    m_cfg (cfg),
    m_phy (cfg.m_radio, cfg.m_cw)
  {
    WIFLX_LOG_FUNCTION (this);
  }

  ~cw_phy_test ()
  {
    WIFLX_LOG_FUNCTION (this);
  }

  virtual void start()
  {
    WIFLX_LOG_FUNCTION (this);

    m_phy.start ();

    m_phy_tx_thread = std::move (std::thread (&wiflx::test::cw_phy::tx_run, &m_phy));
    #ifdef WIFLX_RT_SCHED
      wiflx::common::set_thread_param (m_phy_tx_thread.native_handle(), "PHY TX", SCHED_FIFO, 2, 0);
    #endif
  }

  virtual void stop ()
  {
    m_phy.stop ();
    m_phy_tx_thread.join();
  }

  config m_cfg;
  cw_phy m_phy;
  std::thread m_phy_tx_thread;
};

} // namespace test
} // namespace wiflx

#endif // WIFLX_TEST_CW_PHY_TEST_H
