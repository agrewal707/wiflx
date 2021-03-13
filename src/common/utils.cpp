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

#include "utils.h"

#include <fstream>
#include <random>
#include <chrono>

namespace wiflx {
namespace common {

std::string get_file_content (const char* filename)
{
  std::string content;
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    in.seekg(0, std::ios::end);
    content.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&content[0], content.size());
    in.close();
  }

  return content;
}

int get_random_number (int min, int max)
{
  static std::default_random_engine generator (
    std::chrono::system_clock::now().time_since_epoch().count());

  std::uniform_int_distribution<int> distribution(min, max);
  return distribution (generator);
}

} // namespace common
} // namespace wiflx
