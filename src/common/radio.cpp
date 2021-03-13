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
#include <common/radio.h>

#include <common/log.h>

namespace wiflx {
namespace common {

radio::radio (const config::radio &cfg, pipebuf_cf &rxbuff, pipebuf_cf &txbuff):
  m_cfg (cfg),
  m_rx (rxbuff),
  m_tx (txbuff),
  m_total_rx_samples(0),
  m_total_tx_samples(0)
{
  WIFLX_LOG_FUNCTION (this);

  SoapySDR::Kwargs args =
  {
    {"driver", "plutosdr" },
    {"uri", m_cfg.uri.c_str() },
    {"rx_buf_count", std::to_string(m_cfg.rx_buf_count)},
    {"tx_buf_count", std::to_string(m_cfg.tx_buf_count)}
  };
  for (auto &v : args)
  {
    WIFLX_LOG_DEBUG ("key: {}, val: {}", v.first, v.second);
  }
	m_sdr = SoapySDR::Device::make(args);
	if (!m_sdr)
	{
		WIFLX_LOG_ERROR ("SoapySDR::Device::make failed");
    return;
	}

  // RX
  m_sdr->setSampleRate (SOAPY_SDR_RX, 0, cfg.sampling_frequency);
  m_sdr->setFrequency (SOAPY_SDR_RX, 0, cfg.rxfreq);
  m_sdr->setBandwidth (SOAPY_SDR_RX, 0, cfg.analog_bandwidth);
  m_sdr->setGainMode (SOAPY_SDR_RX, 0, false);
  m_sdr->setGain (SOAPY_SDR_RX, 0, cfg.rxgain);


  SoapySDR::Kwargs rxargs =
  {
    {"bufflen", std::to_string(cfg.rx_buf_size) }
  };
  m_rx_stream = m_sdr->setupStream (SOAPY_SDR_RX, SOAPY_SDR_CF32, std::vector<size_t>(), rxargs);
  if (!m_rx_stream)
  {
    WIFLX_LOG_ERROR ("setup rx stream failed");
    SoapySDR::Device::unmake (m_sdr);
    return;
  }

  // TX
  m_sdr->setSampleRate (SOAPY_SDR_TX, 0, cfg.sampling_frequency);
  m_sdr->setFrequency (SOAPY_SDR_TX, 0, cfg.txfreq);
  m_sdr->setBandwidth (SOAPY_SDR_TX, 0, cfg.analog_bandwidth);
  m_sdr->setGain (SOAPY_SDR_TX, 0, cfg.txgain);


  SoapySDR::Kwargs txargs =
  {
    {"bufflen", std::to_string(cfg.tx_buf_size) }
  };
  m_tx_stream  = m_sdr->setupStream (SOAPY_SDR_TX, SOAPY_SDR_CF32, std::vector<size_t>(), txargs);
  if (!m_tx_stream)
  {
    WIFLX_LOG_ERROR ("setup tx stream failed");
    if (m_rx_stream)
    {
      m_sdr->deactivateStream (m_rx_stream, 0, 0);
      m_sdr->closeStream (m_rx_stream);
    }
    SoapySDR::Device::unmake (m_sdr);
    return;
  }
}

radio::~radio ()
{
  WIFLX_LOG_FUNCTION (this);

  if (m_sdr)
  {
    if (m_rx_stream)
      m_sdr->closeStream (m_rx_stream);

    if (m_tx_stream)
      m_sdr->closeStream (m_tx_stream);

    SoapySDR::Device::unmake (m_sdr);
  }
}

void radio::start ()
{
  WIFLX_LOG_FUNCTION (this);

  if (m_sdr)
  {
    if (m_rx_stream)
      m_sdr->activateStream (m_rx_stream, 0, 0, 0);

    if (m_tx_stream)
      m_sdr->activateStream (m_tx_stream, 0, 0, 0);
  }
}

void radio::stop ()
{
  WIFLX_LOG_FUNCTION (this);

  if (m_sdr)
  {
    if (m_rx_stream)
      m_sdr->deactivateStream (m_rx_stream, 0, 0);

    if (m_tx_stream)
      m_sdr->deactivateStream (m_tx_stream, 0, 0);
  }
}

size_t radio::rx_step ()
{
  size_t retval = 0;

  if (m_rx.writable())
  {
    //WIFLX_LOG_ERROR ("RX STEP {:d}", m_rx.writable());
    int flags = 0;
    long long time_ns = 0;
    std::vector<void*> buffs(1);
    buffs[0] = m_rx.wr();
    retval = m_sdr->readStream(m_rx_stream, buffs.data(), m_rx.writable(), flags, time_ns);
    if (retval < 0)
    {
      WIFLX_LOG_ERROR ("unexpected readStream error: {}", SoapySDR::errToStr(retval));
    }
    else if (retval > 0)
    {
      m_total_rx_samples += retval;
      m_rx.written(retval);
    }
  }
  else
    WIFLX_LOG_ERROR ("UNDERFLOW");

  return retval;
}

void radio::tx_step ()
{
  while (m_tx.readable() > 0)
  {
    //WIFLX_LOG_ERROR ("TX STEP {:d}", m_tx.readable());
    int flags = SOAPY_SDR_END_BURST;
  //  int flags = 0;
    long long txtime_ns = 0; // not used on pluto
    std::vector<void*> buffs(1);
    buffs[0] = m_tx.rd();
    const auto r = m_sdr->writeStream(m_tx_stream, buffs.data(), m_tx.readable(), flags, txtime_ns);
    if (r < 0)
    {
      WIFLX_LOG_ERROR ("unexpected writeStream error: {}", SoapySDR::errToStr(r));
    }
    else
    {
      m_total_tx_samples += r;
      m_tx.read (r);
    }
  }
}

void radio::info (const int direction)
{
  const char *dir = (direction == SOAPY_SDR_RX) ? "RX" : "TX";
  {
    auto str_list = m_sdr->listAntennas(direction, 0);
    WIFLX_LOG_DEBUG("{} antennas: ", dir);
    for(int i = 0; i < str_list.size(); ++i)
      WIFLX_LOG_DEBUG("{},", str_list[i]);
    WIFLX_LOG_DEBUG("\n");
  }
  {
    WIFLX_LOG_DEBUG("{} Gain: {:f}", dir, m_sdr->getGain (direction, 0, ""));
  }
  {
    auto str_list = m_sdr->listGains(direction, 0);
    WIFLX_LOG_DEBUG("{} Gains: ", dir);
    for(int i = 0; i < str_list.size(); ++i)
      WIFLX_LOG_DEBUG("{}, ", str_list[i].c_str());
    WIFLX_LOG_DEBUG("\n");
  }
  {
    auto ranges = m_sdr->getFrequencyRange(direction, 0);
    WIFLX_LOG_DEBUG("{} freq ranges: ", dir);
    for(int i = 0; i < ranges.size(); ++i)
      WIFLX_LOG_DEBUG("[{:g} Hz -> {:g} Hz], ", ranges[i].minimum(), ranges[i].maximum());
    WIFLX_LOG_DEBUG("\n");
  }
}

void radio::stats ()
{
  WIFLX_LOG_DEBUG ("totalRxSamples: {:d}\n", m_total_rx_samples);
  WIFLX_LOG_DEBUG ("totalTxSamples: {:d}\n", m_total_tx_samples);
}

} // namespace common
} // namespace wiflx
