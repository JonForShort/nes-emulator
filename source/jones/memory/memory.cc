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

#include <boost/static_assert.hpp>

using namespace jones;

uint8_t memory::read(const uint16_t address) const {
  for (const auto &i : memory_mappings_) {
    if (address >= i->start_address() && address <= i->end_address()) {
      return i->read(address);
    }
  }
  BOOST_STATIC_ASSERT("unexpected read address");
  return 0;
}

uint16_t memory::read_word(const uint16_t address) const {
  return static_cast<uint16_t>(read(address)) |
         static_cast<uint16_t>(read(address + 1) << 8U);
}

void memory::write(const uint16_t address, uint8_t data) const {
  for (const auto &i : memory_mappings_) {
    if (address >= i->start_address() && address <= i->end_address()) {
      i->write(address, data);
      return;
    }
  }
  BOOST_STATIC_ASSERT("unexpected write address");
}

void memory::map(memory_mappable_ptr mappable) {
  memory_mappings_.emplace_back(std::move(mappable));
}
