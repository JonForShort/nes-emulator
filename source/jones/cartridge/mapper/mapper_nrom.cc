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
#include <boost/core/ignore_unused.hpp>
#include <boost/static_assert.hpp>

#include "mapper_nrom.hh"

using namespace jones;

mapper_nrom::mapper_nrom(const mapper_view &mapper_view)
    : mapper(mapper_view),
      type_(resolve_type(mapper_view.cartridge())),
      mirroring_type_(resolve_mirroring_type(mapper_view.cartridge())),
      use_chrram(mapper_view.cartridge().header()->chrrom_size() <= 0) {
  mapper_view.cpu_memory().map(std::make_unique<memory_mappable_component<mapper_nrom>>(this, 0x8000, 0xFFFF));
  mapper_view.ppu_memory().map(std::make_unique<memory_mappable_component<mapper_nrom>>(this, 0x0000, 0x1FFF));
  if (use_chrram) {
    chrram_.resize(0x2000);
    std::fill(chrram_.begin(), chrram_.end(), 0);
  }
}

auto mapper_nrom::read(const uint16_t address) -> uint8_t {
  if (address >= 0x8000) {
    return read_prg(address);
  } else {
    return read_chr(address);
  }
}

auto mapper_nrom::write(const uint16_t address, const uint8_t data) -> void {
  if (address >= 0x8000) {
    return write_prg(address, data);
  } else {
    return write_chr(address, data);
  }
}

uint8_t mapper_nrom::read_prg(const uint16_t address) const {
  const auto base_prgrom_address = get_cartridge().address() + get_cartridge().header()->prgrom_offset();
  const auto base_address = address - 0x8000;
  switch (type_) {
  case nrom_type::NROM_128: {
    const auto prgrom_size = get_cartridge().header()->prgrom_size();
    const auto address_offset = base_address >= prgrom_size ? base_address - prgrom_size : base_address;
    return *(base_prgrom_address + address_offset);
  }
  case nrom_type::NROM_256: {
    return *(base_prgrom_address + base_address);
  }
  default:
    break;
  }
  BOOST_STATIC_ASSERT("unable to read prg rom");
  return 0;
}

void mapper_nrom::write_prg(const uint16_t address, const uint8_t data) {
  boost::ignore_unused(address, data);
  BOOST_STATIC_ASSERT("unable to write to prg rom");
}

uint8_t mapper_nrom::read_chr(const uint16_t address) const {
  if (use_chrram) {
    return chrram_[address];
  } else {
    return *(get_cartridge().address() + get_cartridge().header()->chrrom_offset() + address);
  }
}

void mapper_nrom::write_chr(const uint16_t address, const uint8_t data) {
  if (use_chrram) {
    chrram_[address] = data;
  } else {
    BOOST_STATIC_ASSERT("unable to write to chr rom");
  }
}