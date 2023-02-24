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
#include <common/profiling.h>
#include <common/utils.h>

#include <iio.h>
#include <ad9361.h>

#define DEBUG_BUFFER_LEN (16384)

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
    {"driver", m_cfg.driver.c_str() },
    {"uri", m_cfg.uri.c_str() },
    {"serial", m_cfg.uri.c_str() },
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
		std::runtime_error ("SoapySDR::Device::make failed");
	}

  if (args["driver"] == "plutosdr" && !cfg.fir_filter_file.empty())
  {
    pluto_load_fir_filter (cfg.fir_filter_file.c_str());
  }

  // RX
  m_sdr->setBandwidth (SOAPY_SDR_RX, 0, cfg.rx_analog_bandwidth);
  m_sdr->setSampleRate (SOAPY_SDR_RX, 0, cfg.sampling_frequency);
  m_sdr->setFrequency (SOAPY_SDR_RX, 0, cfg.rxfreq);
  m_sdr->setGainMode (SOAPY_SDR_RX, 0, false);
  m_sdr->setGain (SOAPY_SDR_RX, 0, cfg.rxgain);

  SoapySDR::Kwargs rxargs =
  {
    {"bufflen", std::to_string(cfg.rx_buf_size) }

  };
  if (args["driver"] == "uhd")
  {
    rxargs.emplace ("num_recv_frames", std::to_string(m_cfg.rx_buf_count));
    rxargs.emplace ("recv_frame_size", std::to_string(cfg.rx_buf_size));
    rxargs.emplace ("fullscale", "4.0");
  }
  else if (args["driver"] == "bladerf")
  {
    rxargs.emplace ("buffers", std::to_string(m_cfg.rx_buf_count));
    rxargs.emplace ("buflen", std::to_string(m_cfg.rx_buf_size));
  }

  m_rx_stream = m_sdr->setupStream (SOAPY_SDR_RX, SOAPY_SDR_CF32, std::vector<size_t>(), rxargs);
  if (!m_rx_stream)
  {
    SoapySDR::Device::unmake (m_sdr);
    throw std::runtime_error ("setup rx stream failed");
  }

  // TX
  m_sdr->setBandwidth (SOAPY_SDR_TX, 0, cfg.tx_analog_bandwidth);
  m_sdr->setSampleRate (SOAPY_SDR_TX, 0, cfg.sampling_frequency);
  m_sdr->setFrequency (SOAPY_SDR_TX, 0, cfg.txfreq);
  m_sdr->setGain (SOAPY_SDR_TX, 0, cfg.txgain);
 
  SoapySDR::Kwargs txargs =
  {
    {"bufflen", std::to_string(cfg.tx_buf_size) }
  };
  if (args["driver"] == "uhd")
  {
    rxargs.emplace ("num_send_frames", std::to_string(m_cfg.tx_buf_count));
    rxargs.emplace ("send_frame_size", std::to_string(cfg.tx_buf_size));
    txargs.emplace ("fullscale", "4.0");
  }
  else if (args["driver"] == "bladerf")
  {
    txargs.emplace ("buffers", std::to_string(m_cfg.tx_buf_count));
    txargs.emplace ("buflen", std::to_string(m_cfg.tx_buf_size));
  }

  m_tx_stream  = m_sdr->setupStream (SOAPY_SDR_TX, SOAPY_SDR_CF32, std::vector<size_t>(), txargs);
  if (!m_tx_stream)
  {
    if (m_rx_stream)
    {
      m_sdr->deactivateStream (m_rx_stream, 0, 0);
      m_sdr->closeStream (m_rx_stream);
    }
    SoapySDR::Device::unmake (m_sdr);
    throw std::runtime_error ("setup tx stream failed");
  }

  #ifdef WIFLX_RADIO_DEBUG_SAMPLES
   m_debug_tx = windowcf_create(DEBUG_BUFFER_LEN);
   m_debug_rx = windowcf_create(DEBUG_BUFFER_LEN);
  #endif
}

radio::~radio ()
{
  WIFLX_LOG_FUNCTION (this);

  stats ();

  #ifdef WIFLX_RADIO_DEBUG_SAMPLES
    dump_debug_data ("/tmp/wiflx_radio_samples.m");
    windowcf_destroy(m_debug_tx);
    windowcf_destroy(m_debug_rx);
  #endif

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
    WIFLX_PROFILING_SCOPE_N("radio_rx_step");
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
      #ifdef WIFLX_RADIO_DEBUG_SAMPLES
        windowcf_write (m_debug_rx, m_rx.wr(), retval);
      #endif

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
    WIFLX_PROFILING_SCOPE_N("radio_tx_step");
    //WIFLX_LOG_ERROR ("TX STEP {:d}", m_tx.readable());
    int flags = SOAPY_SDR_END_BURST;
    //int flags = 0;
    std::vector<void*> buffs(1);
    buffs[0] = m_tx.rd();
    const auto r = m_sdr->writeStream(m_tx_stream, buffs.data(), m_tx.readable(), flags);
    if (r < 0)
    {
      WIFLX_LOG_ERROR ("unexpected writeStream error: {}", SoapySDR::errToStr(r));
    }
    else
    {
      #ifdef WIFLX_RADIO_DEBUG_SAMPLES
        windowcf_write (m_debug_tx, m_tx.rd(), r);
      #endif

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
    WIFLX_LOG_DEBUG("{} Gain: {:f}", dir, m_sdr->getGain (direction, 0));
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

void radio::pluto_load_fir_filter (const char *filename)
{
  struct iio_device *dev = (struct iio_device*) (m_sdr->getNativeDeviceHandle());
  if (!dev)
    throw std::runtime_error ("invalid phy device");

  const auto fir = common::get_file_content (filename);
  const auto lines = std::count (fir.begin(), fir.end(), '\n');
  int taps = lines - 10; // 10 lines for headers
  WIFLX_LOG_DEBUG("fir taps {}", taps);

  auto ret = iio_device_attr_write_raw(dev, "filter_fir_config", fir.data(), fir.size());
  if (ret < 0)
    throw std::runtime_error ("failed to load fir filter");

  auto *chan_rx = iio_device_find_channel(dev, "voltage0", false);
  if (!chan_rx)
    throw std::runtime_error ("failed to get RX channel");

  ret = iio_channel_attr_write_longlong(chan_rx, "filter_fir_en", 1);
  if (ret < 0)
    throw std::runtime_error ("failed to enable fir filter on RX channel");

  auto *chan_tx = iio_device_find_channel(dev, "voltage0", true);
  if (!chan_tx)
    throw std::runtime_error ("failed to get TX channel");

  ret = iio_channel_attr_write_longlong(chan_tx, "filter_fir_en", 1);
  if (ret < 0)
    throw std::runtime_error ("failed to enable fir filter on RX channel");
}

#ifdef WIFLX_RADIO_DEBUG_SAMPLES
void radio::dump_debug_data (const char *_filename)
{
  auto* fid = fopen(_filename,"w");
  if (!fid) {
    WIFLX_LOG_ERROR("could not open {} for writing", _filename);
    return;
  }

  fprintf(fid,"%% %s: auto-generated file", _filename);
  fprintf(fid,"\n\n");
  fprintf(fid,"clear all;\n");
  fprintf(fid,"close all;\n\n");
  fprintf(fid,"n = %u;\n", DEBUG_BUFFER_LEN);

  std::complex<float> *rc;

  // write TX
  fprintf(fid,"tx = zeros(1,n);\n");
  windowcf_read(m_debug_tx, &rc);
  for (unsigned i=0; i<DEBUG_BUFFER_LEN; i++)
    fprintf(fid,"tx(%4u) = %12.4e + j*%12.4e;\n", i+1, std::real(rc[i]), std::imag(rc[i]));
  fprintf(fid,"\n\n");

  // TX figure
  fprintf(fid,"figure('Name', 'TX', 'Color','white','position',[100 100 800 600]);\n");
  fprintf(fid,"plotspec2(tx(1:end), %f, [0 60], [-100 0]);\n", 1/m_cfg.sampling_frequency);

  // write RX
  fprintf(fid,"rx = zeros(1,n);\n");
  windowcf_read(m_debug_rx, &rc);
  for (unsigned i=0; i<DEBUG_BUFFER_LEN; i++)
    fprintf(fid,"rx(%4u) = %12.4e + j*%12.4e;\n", i+1, std::real(rc[i]), std::imag(rc[i]));
  fprintf(fid,"\n\n");

  // RX figure
  fprintf(fid,"figure('Name', 'RX', 'Color','white','position',[100 100 800 600]);\n");
  fprintf(fid,"plotspec2(rx(1:end), %f, [0 6], [-100 0]);\n", 1/m_cfg.sampling_frequency);

  fprintf(fid,"\n\n");
  fclose(fid);

  WIFLX_LOG_INFO("tx/rx samples written to {}", _filename);
}
#endif

} // namespace common
} // namespace wiflx
