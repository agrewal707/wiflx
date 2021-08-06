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

#include <common/log.h>

namespace wiflx {
namespace common {

void log_init (const std::string &logfile)
{
  quill::enable_console_colours();

  auto* file_handler = quill::stdout_handler();
  if (!logfile.empty())
    file_handler = quill::file_handler(logfile, "w");

  file_handler->set_pattern(
    QUILL_STRING("%(ascii_time) %(filename):%(function_name) %(message)"), // log recorder format
    "%H:%M:%S.%Qus", // timestamp format
    quill::Timezone::GmtTime); // timestamp's timezone

  quill::set_default_logger_handler(file_handler);

  quill::start();
}

void log_set_level (const quill::LogLevel level)
{
  quill::get_logger()->set_log_level (level);
}

void log_hexdump(const unsigned char buf[], int len, int dlen)
{
  char line[128];
  for (int i = 0; i < len; i += dlen)
  {
    char *p  = line;
    p += sprintf (p, "0x%04x: ", i);
    for (int j = 0; j < dlen; ++j)
    {
      if ((i+j) < len)
      {
        p += sprintf (p, "%02x ", static_cast<int32_t>(buf[i+j]));
      }
      else
      {
        p += sprintf(p, "    ");
      }
    }

    p += sprintf(p, " ");

    for (int j = 0; j < dlen; ++j)
    {
      if ( (i+j) < len)
      {
        if (isprint(static_cast<int32_t>(buf[i+j])) != 0)
        {
          p += sprintf (p, "%c", buf[i+j]);
        }
        else
        {
          p+= sprintf (p, ".");
        }
      }
    }

    WIFLX_LOG_DEBUG("{}", line);
  }
}

} // namespace common
} // namespace wiflx
