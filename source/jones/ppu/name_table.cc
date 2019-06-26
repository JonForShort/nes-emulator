//
// MIT License
//
// Copyright 2019
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include "name_table.hh"

#include <boost/core/ignore_unused.hpp>

using namespace jones::ppu;

namespace {

constexpr uint16_t mirror_table_mapping[][4]{
    {0, 0, 1, 1},
    {0, 1, 0, 1},
    {0, 0, 0, 0},
    {1, 1, 1, 1},
    {0, 1, 2, 3},
};

constexpr inline auto get_address_offset(uint16_t const address) {
  const auto relative = (address - name_table_memory_begin) % name_table_max_size;
  const auto table = (relative / name_table_size);
  const auto offset = (relative % name_table_size);
  const auto absolute = name_table_memory_begin + mirror_table_mapping[2][table] * name_table_size + offset;
  return absolute % (name_table_size * 2);
}

} // namespace

auto name_table::peek(uint16_t const address) const -> uint8_t {
  return read(address);
}

auto name_table::read(uint16_t const address) const -> uint8_t {
  return name_table_[get_address_offset(address)];
}

auto name_table::write(uint16_t const address, uint8_t const data) -> void {
  name_table_[get_address_offset(address)] = data;
}
