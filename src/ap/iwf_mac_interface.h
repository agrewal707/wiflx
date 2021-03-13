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

#ifndef WIFLX_AP_IWF_MAC_INTERFACE_H
#define WIFLX_AP_IWF_MAC_INTERFACE_H

#include <common/messages/mac.pb.h>

namespace wiflx {
namespace ap {
namespace iwf_mac {

struct sap_user
{
  virtual ~sap_user () {};
  virtual void on_receive (const uint16_t rid, const std::string &ipdu) = 0;
};

struct sap_provider
{
  virtual ~sap_provider () {};
  virtual void send (const uint16_t rid, const uint8_t tid, const uint32_t fid, std::string &&msdu) = 0;
};

} // namespace iwf_mac
} // namespace ap
} // namespace wiflx

#endif // WIFLX_AP_IWF_MAC_INTERFACE_H
