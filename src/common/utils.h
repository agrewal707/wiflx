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
#ifndef WIFLX_COMMON_UTILS_H
#define WIFLX_COMMON_UTILS_H

#include <string>

namespace wiflx {
namespace common {

std::string get_file_content (const char *filename);

int get_random_number (int min, int max);

} // namespace common
} // namespace wiflx

#include <array>

namespace std
{
  template<class T, size_t N>
  class hash<std::array<T, N>>
  {
  public:
    auto operator() (const std::array<T, N>& key) const
    {
      size_t result = 0;
      for(size_t i = 0; i < N; ++i)
        result = result * 31 + m_hasher(key[i]);
      return result;
    }
  private:
    std::hash<T> m_hasher;
  };
}

#endif // WIFLX_COMMON_UTILS_H
