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
#ifndef WIFLX_TEST_CW_PHY_H
#define WIFLX_TEST_CW_PHY_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <common/io_context.h>
#include <common/radio.h>
#include <test/config.h>

namespace wiflx {
namespace test {

class cw_phy
{
public:
  cw_phy (const common::config::radio &rcfg,
          const config::cw_test &cfg);
  ~cw_phy ();

  void start ();
  void stop ();
  void tx_run ();

private:
  wiflx::common::pipebuf_cf m_rxbuff;
  wiflx::common::pipebuf_cf m_txbuff;
  wiflx::common::radio m_radio;
  common::pipewriter_cf m_tx;
  std::vector<std::complex<float>> m_buff;
  bool m_stopped;
};

} // namespace test
} // namespace wiflx

#endif // WIFLX_TEST_CW_PHY_H
