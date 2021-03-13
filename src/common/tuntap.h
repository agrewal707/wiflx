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

#ifndef WIFLX_COMMON_TUNTAP_H
#define WIFLX_COMMON_TUNTAP_H

#include <common/io_context.h>

namespace wiflx {
namespace common {

class tuntap
{
public:
  tuntap (wiflx::common::io_context &ioc, const std::string &devname = std::string());
  ~tuntap ();

  template <typename MutableBufferSequence, typename Handler>
  void async_receive (const MutableBufferSequence &buffers, const Handler &handler)
  {
    m_device.async_read_some (buffers, handler);
  }

  template <typename ConstBufferSequence, typename Handler>
  void async_send (const ConstBufferSequence &buffers, const Handler &handler)
  {
    m_device.async_write_some (buffers, handler);
  }

  void cancel ()
  {
    m_device.cancel ();
  }

private:
  void init (const std::string &devname);
  boost::asio::posix::stream_descriptor m_device;
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_TUNTAP_H
