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

#ifndef WIFLX_COMMON_TIMER_H
#define WIFLX_COMMON_TIMER_H

#ifdef WIFLX_SIMULATION
//TODO
#else
#include <boost/asio/steady_timer.hpp>
#endif

namespace wiflx {
namespace common {

#ifdef WIFLX_SIMULATION
using timer = wiflx::sim::timer;
#else
using timer = boost::asio::steady_timer;
#endif

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_TIMER_H
