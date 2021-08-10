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

#include <test/cw_phy.h>

#include <common/log.h>
#include <common/gpio.h>

namespace wiflx {
namespace test {

using std::mutex;
using std::unique_lock;
using std::lock_guard;

cw_phy::cw_phy (
  const common::config::radio &rcfg,
  const config::cw_test &cfg) :
  m_rxbuff ("RX BUFFER", 16834),
  m_txbuff ("TX BUFFER", 16834),
  m_radio (rcfg, m_rxbuff, m_txbuff),
  m_tx (m_txbuff, rcfg.tx_buf_size),
  m_stopped (true)
{
  WIFLX_LOG_FUNCTION (this);

  m_radio.info (SOAPY_SDR_RX);
  m_radio.info (SOAPY_SDR_TX);

  m_buff.reserve (rcfg.tx_buf_size);
  for (int i = 0; i < rcfg.tx_buf_size; ++i)
  {
    const auto theta = 2*M_PI*cfg.tone_freq*i/rcfg.sampling_frequency;
    m_buff.emplace_back (cosf(theta), sinf(theta));
  }
}

cw_phy::~cw_phy ()
{
  WIFLX_LOG_FUNCTION (this);
}

void cw_phy::start ()
{
  WIFLX_LOG_FUNCTION (this);

  if (m_stopped)
  {
    m_stopped = false;
    m_radio.start ();
  }
}

void cw_phy::stop ()
{
  WIFLX_LOG_FUNCTION (this);

  if (!m_stopped)
  {
    m_stopped = true;
    m_radio.stop ();
  }
}

void cw_phy::tx_run ()
{
  WIFLX_LOG_FUNCTION (this);

  while (!m_stopped)
  {
    if (m_buff.size() <= m_tx.writable())
      for (int i = 0; i < m_buff.size(); ++i)
        m_tx.write (m_buff[i]);

    WIFLX_GPIO_SET(common::gpio::GPIO_1);
    m_radio.tx_step();
    WIFLX_GPIO_CLEAR(common::gpio::GPIO_1);
  }
}

} // namespace test
} // namespace wiflx
