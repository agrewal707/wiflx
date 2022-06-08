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

#include <common/ofdm.h>

#include <common/log.h>
#include <common/gpio.h>
#include <common/profiling.h>
#include <sstream>

namespace wiflx {
namespace common {

//
// ofdm_tx
//
ofdm_tx::ofdm_tx (const config::ofdm &cfg, pipebuf_cf &buff, radio &r) :
  m_cfg (cfg),
  m_tx (buff, m_cfg.M+m_cfg.cp_len),
  m_radio (r)
{
  WIFLX_LOG_FUNCTION (this);

  ofdmflexframegenprops_s fgprops;
  ofdmflexframegenprops_init_default(&fgprops);
  fgprops.mod_scheme      = liquid_getopt_str2mod(m_cfg.ms.c_str()); // set the modulation scheme
  fgprops.fec0            = liquid_getopt_str2fec(m_cfg.fec0.c_str()); // set the inner FEC scheme
  fgprops.fec1            = liquid_getopt_str2fec(m_cfg.fec1.c_str()); // set the outer FEC scheme
  fgprops.check           = liquid_getopt_str2crc(m_cfg.check.c_str()); // set the error-detection scheme

  // subcarrier allocation (null/pilot/data)
  unsigned char p[m_cfg.M];
  if (m_cfg.p.empty())
    ofdmframe_init_default_sctype (m_cfg.M, p);
  else
    memcpy (p, m_cfg.p.data(), m_cfg.M);

  m_fg = ofdmflexframegen_create (m_cfg.M, m_cfg.cp_len, m_cfg.taper_len, p, &fgprops);
  ofdmflexframegen_set_header_len (m_fg, 0);
}

ofdm_tx::~ofdm_tx ()
{
  WIFLX_LOG_FUNCTION (this);

  ofdmflexframegen_destroy (m_fg);
}

void ofdm_tx::start ()
{
  WIFLX_LOG_FUNCTION (this);

  ofdmflexframegen_print(m_fg);
}

void ofdm_tx::stop ()
{
  WIFLX_LOG_FUNCTION (this);
}

void ofdm_tx::send (const std::string &psdu)
{
  WIFLX_LOG_FUNCTION (this);

  bool frame_complete = false;
  {
    WIFLX_PROFILING_SCOPE_N("ofdm_tx_prep");
    WIFLX_GPIO_SET(common::gpio::GPIO_2);
    ofdmflexframegen_assemble (m_fg, nullptr, (const uint8_t*) (psdu.data()), psdu.size());
    //ofdmflexframegen_print (m_fg);
    while (!frame_complete)
    {
      const auto buff_len = m_cfg.M+m_cfg.cp_len;
      if (buff_len <= m_tx.writable())
      {
        frame_complete = ofdmflexframegen_write (m_fg, m_tx.wr(), buff_len);
        m_tx.written (buff_len);
      }
      else
      {
        WIFLX_LOG_ERROR ("OVERFLOW");
        frame_complete = true;
      }
    }
    WIFLX_GPIO_CLEAR(common::gpio::GPIO_2);
  }

  if (frame_complete)
  {
    WIFLX_PROFILING_SCOPE_N("ofdm_tx_send");
    WIFLX_GPIO_SET(common::gpio::GPIO_1);
    m_radio.tx_step();
    WIFLX_GPIO_CLEAR(common::gpio::GPIO_1);
    ofdmflexframegen_reset (m_fg);
  }
}

//
// ofdm_rx
//
ofdm_rx::ofdm_rx (const config::ofdm &cfg, pipebuf_cf &buff, radio &r, listener *l) :
  m_cfg (cfg),
  m_rx (buff),
  m_radio (r),
  m_listener (l)
{
  WIFLX_LOG_FUNCTION (this);

  // subcarrier allocation (null/pilot/data)
  unsigned char p[m_cfg.M];
  if (m_cfg.p.empty())
    ofdmframe_init_default_sctype (m_cfg.M, p);
  else
    memcpy (p, m_cfg.p.data(), m_cfg.M);

  ofdmframe_print_sctype (p, m_cfg.M);
  m_fs = ofdmflexframesync_create(m_cfg.M, m_cfg.cp_len, m_cfg.taper_len, p, on_recv, this);
  ofdmflexframesync_set_header_len (m_fs, 0);
}

ofdm_rx::~ofdm_rx ()
{
  WIFLX_LOG_FUNCTION (this);

#ifdef WIFLX_OFDM_DEBUG
  ofdmflexframesync_debug_print (m_fs, "/tmp/ofdm_rx_debug.m");
#endif

  ofdmflexframesync_destroy (m_fs);
}

void ofdm_rx::start ()
{
  WIFLX_LOG_FUNCTION (this);

  ofdmflexframesync_print(m_fs);
#ifdef WIFLX_OFDM_DEBUG
  ofdmflexframesync_debug_enable (m_fs);
#endif
}

void ofdm_rx::stop ()
{
  WIFLX_LOG_FUNCTION (this);
}

void ofdm_rx::step ()
{
  if (m_radio.rx_step () > 0)
  {
    WIFLX_PROFILING_SCOPE_N("ofdm_rx_execute");
    ofdmflexframesync_execute (m_fs, m_rx.rd(), m_rx.readable());
    m_rx.read (m_rx.readable());
  }
}

int ofdm_rx::on_recv(
  unsigned char    *_header,
  int              _header_valid,
  unsigned char    *_payload,
  unsigned int     _payload_len,
  int              _payload_valid,
  framesyncstats_s _stats,
  void             *_userdata)
{
  ofdm_rx *rx = (ofdm_rx*)_userdata;

  #if 0
  log_hexdump (_payload, _payload_len, 16);
  #endif

  WIFLX_PROFILING_SCOPE_N("ofdm_rx_packet");

  WIFLX_GPIO_SET(common::gpio::GPIO_0);

  WIFLX_LOG_DEBUG("evm: {:f} rssi: {:f} cfo: {:f} valid: {:d}", _stats.evm, _stats.rssi, _stats.cfo, _payload_valid);
  rx->m_listener->on_receive (_header, _header_valid, _payload, _payload_len, _payload_valid, _stats);

  WIFLX_GPIO_CLEAR(common::gpio::GPIO_0);

  return 0;
}

} // namespace common
} // namespace wiflx
