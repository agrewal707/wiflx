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

#ifndef WIFLX_COMMON_GPIO_H
#define WIFLX_COMMON_GPIO_H

#include <config.h>

#ifdef WIFLX_GPIO_DEBUG
#define WIFLX_GPIO_INIT() wiflx::common::gpio::init()
#define WIFLX_GPIO_SET(id) wiflx::common::gpio::set(id)
#define WIFLX_GPIO_CLEAR(id) wiflx::common::gpio::clear(id)
#else
#define WIFLX_GPIO_INIT()
#define WIFLX_GPIO_SET(id)
#define WIFLX_GPIO_CLEAR(id)
#endif

namespace wiflx {
namespace common {

struct gpio
{
	enum ID
	{
		GPIO_0 = 0,
		GPIO_1 = 1,
		GPIO_2 = 2
	};

	static void init ();
	static void set (const ID id);
	static void clear (const ID id);
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_GPIO_H
