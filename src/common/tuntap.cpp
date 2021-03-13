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

#include "tuntap.h"

#include <linux/if.h>
#include <linux/if_tun.h>
#include <common/log.h>

namespace wiflx {
namespace common {

tuntap::tuntap(wiflx::common::io_context &ioc, const std::string &devname):
  m_device (ioc)
{
  WIFLX_LOG_FUNCTION(this);

  init (devname);
}

tuntap::~tuntap()
{
  WIFLX_LOG_FUNCTION(this);
}

void tuntap::init (const std::string &devname)
{
  WIFLX_LOG_FUNCTION(this << devname);

  const int max_size = 16;
  char dev[max_size];
  memset (dev, 0, max_size);
  if (!devname.empty())
  {
    if (devname.size () > max_size-1)
      throw std::runtime_error ("given device name length exceeded");
    strncpy (dev, devname.c_str(), max_size);
  }

  int fd = -1;
  if ((fd = open("/dev/net/tap", O_RDWR)) < 0)
  {
    throw std::runtime_error (std::string("cannot open TUN/TAP device: ")+strerror(errno));
  }

  struct ifreq ifr;
  memset (&ifr, 0, sizeof(ifr));
  // Flags: IFF_TUN   - TUN device (no Ethernet headers)
  //        IFF_TAP   - TAP device
  //       IFF_NO_PI - Do not provide packet information
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
  strncpy(ifr.ifr_name, dev, IFNAMSIZ);

  if (ioctl(fd, TUNSETIFF, (void *) &ifr) < 0)
  {
    close(fd);
    throw std::runtime_error (std::string("TUNSETIFF ioctl failed:" )+strerror(errno));
  }

  strcpy (dev, ifr.ifr_name);

  boost::system::error_code ec;
  m_device.assign (fd, ec);
  if (ec)
  {
    close (fd);
    throw std::runtime_error ("failed to assign fd");
  }

  WIFLX_LOG_INFO ("opened device: {}", dev);
}

} // namespace common
} // namespace wiflx
