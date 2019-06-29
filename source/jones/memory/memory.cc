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
#include "memory.hh"

#include <boost/format.hpp>
#include <boost/static_assert.hpp>

using namespace jones;

auto memory::peek(uint16_t const address) const -> uint8_t {
  for (const auto &i : memory_mappings_) {
    if (address >= i->start_address() && address <= i->end_address()) {
      return i->peek(address);
    }
  }
  BOOST_STATIC_ASSERT("unexpected peek address");
  return 0;
}

auto memory::peek_word(uint16_t const address) const -> uint16_t {
  return static_cast<uint16_t>(peek(address)) |
         static_cast<uint16_t>(peek(address + 1) << 8U);
}

auto memory::read(const uint16_t address) const -> uint8_t {
  for (const auto &i : memory_mappings_) {
    if (address >= i->start_address() && address <= i->end_address()) {
      return i->read(address);
    }
  }
  BOOST_STATIC_ASSERT("unexpected read address");
  return 0;
}

auto memory::read_word(uint16_t const address) const -> uint16_t {
  return static_cast<uint16_t>(read(address)) |
         static_cast<uint16_t>(read(address + 1) << 8U);
}

auto memory::write(uint16_t const address, uint8_t const data) const -> void {
  for (const auto &i : memory_mappings_) {
    if (address >= i->start_address() && address <= i->end_address()) {
      i->write(address, data);
      return;
    }
  }
  BOOST_STATIC_ASSERT("unexpected write address");
}

auto memory::map(memory_mappable_ptr mappable) -> void {
  memory_mappings_.emplace_back(std::move(mappable));
}

auto memory::print(std::ostream &out) const -> void {
  for (const auto &mapping : memory_mappings_) {
    out << boost::format("[%s] : [%d$#x] - [%d$#x] : [%s]") % tag_ % mapping->start_address() % mapping->end_address() % mapping->tag();
    out << std::endl;
  }
}
