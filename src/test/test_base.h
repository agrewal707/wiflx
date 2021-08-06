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
#ifndef WIFLX_TEST_TEST_BASE_H
#define WIFLX_TEST_TEST_BASE_H

#include <common/object_factory.h>
#include <test/config.h>

namespace wiflx {
namespace test {

struct test_base
{
  test_base (const wiflx::test::config &cfg) {}
  virtual ~test_base () {}
  virtual void start () = 0;
  virtual void stop () = 0;

  using Factory = wiflx::common::object_factory<test_base,const wiflx::test::config&>;
	static Factory& get_factory ()
	{
		static Factory factory;
		return factory;
	}
};

} // namespace test
} // namespace wiflx

#endif // WIFLX_TEST_TEST_BASE_H
