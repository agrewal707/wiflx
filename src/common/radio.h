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

#ifndef WIFLX_COMMON_RADIO_H
#define WIFLX_COMMON_RADIO_H

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>

#include <config.h>
#include <common/config.h>
#include <common/pipebuf.h>
#include <liquid/liquid.h>

namespace wiflx {
namespace common {

class radio
{
public:
  radio (const config::radio &cfg, pipebuf_cf &rxbuff, pipebuf_cf &txbuff);
  ~radio ();

  void start();
  void stop ();
  size_t rx_step ();
  void tx_step ();
  void info (const int direction);
  void stats ();

private:
  void pluto_load_fir_filter (const char *filename);

  const config::radio &m_cfg;
  pipewriter_cf m_rx;
  pipereader_cf m_tx;
  SoapySDR::Device *m_sdr;
  SoapySDR::Stream *m_rx_stream;
  SoapySDR::Stream *m_tx_stream;
  long long m_total_rx_samples;
  long long m_total_tx_samples;

  #ifdef WIFLX_RADIO_DEBUG_SAMPLES
  void dump_debug_data (const char *_filename);
  windowcf m_debug_tx;
  windowcf m_debug_rx;
  #endif
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_RADIO_H
