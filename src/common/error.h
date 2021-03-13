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

#ifndef WIFLX_COMMON_ERROR_H
#define WIFLX_COMMON_ERROR_H

#include <boost/system/error_code.hpp>

#ifndef WIFLX_SIMULATION
#include <boost/asio/error.hpp>
#endif

namespace wiflx {
namespace common {
namespace error
{
#ifdef WIFLX_SIMULATION
// TODO
#else
using boost::asio::error::operation_aborted;
#endif

} // namespace error
} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_ERROR_H
