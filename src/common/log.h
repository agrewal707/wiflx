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
#ifndef WIFLX_COMMON_LOG_H
#define WIFLX_COMMON_LOG_H

#include <quill/Quill.h>

#define WIFLX_LOG_INFO(fmt, ...) LOG_INFO(quill::get_logger(), fmt, ##__VA_ARGS__)
#define WIFLX_LOG_ERROR(fmt, ...) LOG_ERROR(quill::get_logger(), fmt, ##__VA_ARGS__)
#define WIFLX_LOG_WARNING(fmt, ...) LOG_WARNING(quill::get_logger(), fmt, ##__VA_ARGS__)
#define WIFLX_LOG_CRITICAL(fmt, ...) LOG_CRITICAL(quill::get_logger(), fmt, ##__VA_ARGS__)
#define WIFLX_LOG_DEBUG(fmt, ...) LOG_DEBUG(quill::get_logger(), fmt, ##__VA_ARGS__)
#define WIFLX_LOG_TRACE_L1(fmt, ...) LOG_TRACE_L1(quill::get_logger(), fmt, ##__VA_ARGS__)

#define DO_WIFLX_LOG_FUNCTION(msg) \
  do { \
    std::stringstream tmp; \
    wiflx::common::parameter_logger p(&tmp); \
    p << msg; \
    WIFLX_LOG_DEBUG ("{}",tmp.str()); \
  } while (false)

#if defined(WIFLX_TRACE)
#include <sstream>
#include <iostream>
#define WIFLX_LOG_FUNCTION(msg) DO_WIFLX_LOG_FUNCTION(msg)
#else // WIFLX_TRACE
#define WIFLX_LOG_FUNCTION(msg)
#endif

#include <cstdlib>
#define WIFLX_ASSERT(condition, message) \
  do { \
    if (!(condition)) { \
      WIFLX_LOG_ERROR ("assert failed, cond: {}", #condition); \
      abort (); \
    } \
  } while (false)

namespace wiflx {
namespace common {

extern void log_init (const std::string &logfile = std::string());
extern void log_set_level (const quill::LogLevel level);
extern void log_hexdump (const unsigned char buf[], int len, int dlen);

#if defined(WIFLX_TRACE)
class parameter_logger: public std::ostream
{
public:
  explicit parameter_logger (std::ostream* os) :
    std::basic_ostream<char>(),
    m_os (os),
    m_item_number (0)
  {}

  template<typename T>
  parameter_logger& operator<< (T param)
  {
    switch (m_item_number)
    {
    case 0:
      // first parameter
      (*m_os) << param;
      break;
    default:
      // parameter following a previous parameter
      (*m_os) << "," << param;
      break;
    }
    m_item_number++;
    return *this;
  }

private:
    std::ostream* m_os;
    int32_t m_item_number;
};
#endif // WIFLX_TRACE

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_LOG_H
