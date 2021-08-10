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

#include <config.h>
#include <common/log.h>
#include <common/gpio.h>
#include <common/utils.h>
#include <common/io_context.h>
#include <test/config_json.h>
#include <test/test_base.h>

int main (int C, char *V[])
{
  using json = nlohmann::json;

  if (C < 2)
  {
    fprintf(stderr, "Usage: %s test.cfg [logfile]\n", V[0]);
    exit(-1);
  }

  try
	{
    if (C < 3)
      wiflx::common::log_init ();
    else
      wiflx::common::log_init (std::string(V[2]));

    wiflx::common::log_set_level (quill::LogLevel::Debug);

    WIFLX_GPIO_INIT();

    WIFLX_LOG_DEBUG ("TEST STARTED");

    const auto cfgstr = wiflx::common::get_file_content (V[1]);
    const json j = json::parse (cfgstr, nullptr, true, true);
    const auto cfg = j.get<wiflx::test::config> ();

    auto &test_factory = wiflx::test::test_base::get_factory ();

    std::unique_ptr<wiflx::test::test_base> t;
    if (cfg.m_type == wiflx::test::config::CW_PHY_TEST)
      t.reset (test_factory.create_impl ("cw_phy_test", cfg));
    else if (cfg.m_type == wiflx::test::config::OFDM_PHY_TEST)
      t.reset (test_factory.create_impl ("ofdm_phy_test", cfg));
    else
    {
      WIFLX_LOG_ERROR("unknown test");
      return -1;
    }
    t->start ();

    wiflx::common::io_context ioc;
    boost::asio::signal_set signals (ioc, SIGINT, SIGTERM);
    signals.async_wait (
      [&] (const boost::system::error_code &ec, int signal)
        {
          t->stop ();
          ioc.stop ();
        });

    ioc.run ();
  }
  catch (const std::exception &ex)
  {
    WIFLX_LOG_ERROR ("{}", ex.what());
  }

  return 0;
}
