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

#include <common/sc.h>

#include <common/log.h>
#include <common/gpio.h>
#include <common/profiling.h>
#include <sstream>

namespace wiflx {
namespace common {

//
// sc_tx
//
sc_tx::sc_tx (const config::sc &cfg, pipebuf_cf &buff, radio &r) :
  m_cfg (cfg),
  m_tx (buff, 1),
  m_radio (r)
{
  WIFLX_LOG_FUNCTION (this);

  flexframegenprops_s h_props;
  h_props.mod_scheme      = LIQUID_MODEM_QPSK;    // set the modulation scheme
  h_props.fec0            = LIQUID_FEC_NONE;  // set the inner FEC scheme
  //h_props.fec0            = LIQUID_FEC_CONV_V27;  // set the inner FEC scheme
  h_props.fec1            = LIQUID_FEC_NONE;      // set the outer FEC scheme
  h_props.check           = LIQUID_CRC_32;      // set the error-detection scheme

  flexframegenprops_s d_props;
  d_props.mod_scheme      = liquid_getopt_str2mod(m_cfg.ms.c_str()); // set the modulation scheme
  d_props.fec0            = liquid_getopt_str2fec(m_cfg.fec0.c_str()); // set the inner FEC scheme
  d_props.fec1            = liquid_getopt_str2fec(m_cfg.fec1.c_str()); // set the outer FEC scheme
  d_props.check           = liquid_getopt_str2crc(m_cfg.check.c_str()); // set the error-detection scheme

  m_fg = flexframegen_create (&d_props);
  flexframegen_set_header_len (m_fg, 0);
  flexframegen_set_header_props (m_fg, &h_props);
}

sc_tx::~sc_tx ()
{
  WIFLX_LOG_FUNCTION (this);

  flexframegen_destroy (m_fg);
}

void sc_tx::start ()
{
  WIFLX_LOG_FUNCTION (this);

  flexframegen_print(m_fg);
}

void sc_tx::stop ()
{
  WIFLX_LOG_FUNCTION (this);
}

void sc_tx::send (const std::string &psdu)
{
  WIFLX_LOG_FUNCTION (this);

  bool frame_complete = false;
  {
    WIFLX_PROFILING_SCOPE_N("sc_tx_prep");
    WIFLX_GPIO_SET(common::gpio::GPIO_2);
    flexframegen_assemble (m_fg, nullptr, (const uint8_t*) (psdu.data()), psdu.size());
    //flexframegen_print (m_fg);
    while (!frame_complete)
    {
      const auto buff_len = 1;
      if (buff_len <= m_tx.writable())
      {
        frame_complete = flexframegen_write_samples (m_fg, m_tx.wr(), buff_len);
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
    WIFLX_PROFILING_SCOPE_N("sc_tx_send");
    WIFLX_GPIO_SET(common::gpio::GPIO_1);
    m_radio.tx_step();
    WIFLX_GPIO_CLEAR(common::gpio::GPIO_1);
    flexframegen_reset (m_fg);
  }
}

//
// sc_rx
//
sc_rx::sc_rx (const config::sc &cfg, pipebuf_cf &buff, radio &r, listener *l) :
  m_cfg (cfg),
  m_rx (buff),
  m_radio (r),
  m_listener (l)
{
  WIFLX_LOG_FUNCTION (this);

  flexframegenprops_s h_props;
  h_props.mod_scheme      = LIQUID_MODEM_QPSK;    // set the modulation scheme
  //h_props.fec0            = LIQUID_FEC_CONV_V27;  // set the inner FEC scheme
  h_props.fec0            = LIQUID_FEC_NONE;  // set the inner FEC scheme
  h_props.fec1            = LIQUID_FEC_NONE;      // set the outer FEC scheme
  h_props.check           = LIQUID_CRC_32;        // set the error-detection scheme

  m_fs = flexframesync_create(on_recv, this);
  flexframesync_set_header_len (m_fs, 0);
  flexframesync_set_header_props (m_fs, &h_props);
}

sc_rx::~sc_rx ()
{
  WIFLX_LOG_FUNCTION (this);

#ifdef WIFLX_SC_DEBUG
  flexframesync_debug_print (m_fs, "/tmp/sc_rx_debug.m");
#endif

  flexframesync_destroy (m_fs);
}

void sc_rx::start ()
{
  WIFLX_LOG_FUNCTION (this);

  flexframesync_print(m_fs);
#ifdef WIFLX_SC_DEBUG
  flexframesync_debug_enable (m_fs);
#endif
}

void sc_rx::stop ()
{
  WIFLX_LOG_FUNCTION (this);
}

void sc_rx::step ()
{
  if (m_radio.rx_step () > 0)
  {
    WIFLX_PROFILING_SCOPE_N("sc_rx_execute");
    flexframesync_execute (m_fs, m_rx.rd(), m_rx.readable());
    m_rx.read (m_rx.readable());
  }
}

int sc_rx::on_recv(
  unsigned char    *_header,
  int              _header_valid,
  unsigned char    *_payload,
  unsigned int     _payload_len,
  int              _payload_valid,
  framesyncstats_s _stats,
  void             *_userdata)
{
  sc_rx *rx = (sc_rx*)_userdata;

  #if 0
  log_hexdump (_payload, _payload_len, 16);
  #endif

  WIFLX_PROFILING_SCOPE_N("sc_rx_packet");

  WIFLX_GPIO_SET(common::gpio::GPIO_0);

  WIFLX_LOG_DEBUG("evm: {:f} rssi: {:f} cfo: {:f} valid: {:d}", _stats.evm, _stats.rssi, _stats.cfo, _payload_valid);
  rx->m_listener->on_receive (_header, _header_valid, _payload, _payload_len, _payload_valid, _stats);

  WIFLX_GPIO_CLEAR(common::gpio::GPIO_0);

  return 0;
}

} // namespace common
} // namespace wiflx
