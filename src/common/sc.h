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

#ifndef WIFLX_COMMON_SC_H
#define WIFLX_COMMON_SC_H

#include <math.h>
#include <complex>
#include <liquid/liquid.h>

#include <config.h>
#include <common/radio.h>

namespace wiflx {
namespace common {

class sc_tx
{
public:
  sc_tx (const config::sc &cfg, pipebuf_cf &buff, radio &r);
  ~sc_tx ();

  void start ();
  void stop ();
  void send (const std::string &psdu);

private:
  const config::sc &m_cfg;
  flexframegen m_fg;
  pipewriter_cf m_tx;
  radio &m_radio;
};

class sc_rx
{
public:
  struct listener
  {
    virtual ~listener () {};

    virtual void on_receive (
      unsigned char     *_header,
      int                _header_valid,
      unsigned char     *_payload,
      unsigned int       _payload_len,
      int                _payload_valid,
      framesyncstats_s   _stats) = 0;
  };

  sc_rx (const config::sc &cfg, pipebuf_cf &buff, radio &r, listener *l);
  ~sc_rx ();

  void start ();
  void stop ();
  void step ();

private:
  static int on_recv (
    unsigned char     *_header,
    int               _header_valid,
    unsigned char     *_payload,
    unsigned int       _payload_len,
    int                _payload_valid,
    framesyncstats_s   _stats,
    void              *_userdata);

  const config::sc &m_cfg;
  radio &m_radio;
  listener *m_listener;
  flexframesync m_fs;
  pipereader_cf m_rx;
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_SC_H
