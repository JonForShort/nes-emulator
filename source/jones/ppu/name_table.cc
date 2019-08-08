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
#include "configuration.hh"

using namespace jones::ppu;
using namespace jones::configuration;

namespace {

constexpr uint8_t default_mirror_mode = static_cast<uint8_t>(mirror_mode::MIRROR_MODE_VERTICAL);

constexpr uint16_t mirror_table_mapping[][4]{
    {0, 0, 1, 1},
    {0, 1, 0, 1},
    {0, 0, 0, 0},
    {1, 1, 1, 1},
    {0, 1, 2, 3},
};

constexpr auto get_address_offset(uint16_t const address, uint8_t const mirror) {
  auto const relative = (address - name_table_memory_begin) % name_table_max_size;
  auto const table = (relative / name_table_size);
  auto const offset = (relative % name_table_size);
  auto const absolute = name_table_memory_begin + mirror_table_mapping[mirror][table] * name_table_size + offset;
  return absolute % (name_table_size * 2);
}

auto get_mirror_mode() -> uint8_t {
  return configuration::instance().get<uint8_t>(property::PROPERTY_MIRROR_MODE, default_mirror_mode);
}

} // namespace

auto name_table::peek(uint16_t const address) const -> uint8_t {
  return read(address);
}

auto name_table::read(uint16_t const address) const -> uint8_t {
  auto const adjusted_address = get_address_offset(address, get_mirror_mode());
  return name_table_[adjusted_address];
}

auto name_table::write(uint16_t const address, uint8_t const data) -> void {
  auto const adjusted_address = get_address_offset(address, get_mirror_mode());
  name_table_[adjusted_address] = data;
}
