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
      use_chrram(mapper_view.cartridge().header()->chrrom_size() <= 0),
      prgrom_(get_cartridge().address() + get_cartridge().header()->prgrom_offset()),
      prgrom_size_(get_cartridge().header()->prgrom_size()),
      sram_(0x2000, 0) {
  mapper_view.cpu_memory().map(std::make_unique<memory_mappable_component<mapper_nrom>>(this, "mapper_nrom", 0x6000, 0xFFFF));
  mapper_view.ppu_memory().map(std::make_unique<memory_mappable_component<mapper_nrom>>(this, "mapper_nrom", 0x0000, 0x1FFF));
  if (use_chrram) {
    chrram_.resize(0x2000);
    std::fill(chrram_.begin(), chrram_.end(), 0);
  }
}

auto mapper_nrom::peek(uint16_t const address) const -> uint8_t {
  return read(address);
}

auto mapper_nrom::read(uint16_t const address) const -> uint8_t {
  if (address < 0x2000) {
    return read_chr(address);
  } else if (address >= 0x8000) {
    return read_prg(address);
  } else if (address >= 0x6000) {
    return read_sram(address);
  }
  BOOST_STATIC_ASSERT("unexpected read for mapper nrom");
  return 0;
}

auto mapper_nrom::write(uint16_t const address, uint8_t const data) -> void {
  if (address < 0x2000) {
    return write_chr(address, data);
  } else if (address >= 0x8000) {
    return write_prg(address, data);
  } else if (address >= 0x6000) {
    return write_sram(address, data);
  }
  BOOST_STATIC_ASSERT("unexpected write for mapper nrom");
}

auto mapper_nrom::read_prg(uint16_t const address) const -> uint8_t {
  switch (type_) {
  case nrom_type::NROM_128: {
    const auto offset = address - 0x8000;
    const auto index = offset >= prgrom_size_ ? offset - prgrom_size_ : offset;
    const auto data = prgrom_[index];
    return data;
  }
  case nrom_type::NROM_256: {
    const auto offset = address - 0x8000;
    const auto data = prgrom_[offset];
    return data;
  }
  default:
    break;
  }
  BOOST_STATIC_ASSERT("unable to read prg rom");
  return 0;
}

auto mapper_nrom::write_prg(uint16_t const address, uint8_t const data) -> void {
  boost::ignore_unused(address, data);
  BOOST_STATIC_ASSERT("unable to write to prg rom");
}

auto mapper_nrom::read_chr(const uint16_t address) const -> uint8_t {
  if (use_chrram) {
    return chrram_[address];
  } else {
    return *(get_cartridge().address() + get_cartridge().header()->chrrom_offset() + address);
  }
}

auto mapper_nrom::write_chr(uint16_t const address, uint8_t const data) -> void {
  if (use_chrram) {
    chrram_[address] = data;
  } else {
    BOOST_STATIC_ASSERT("unable to write to chr rom");
  }
}

auto mapper_nrom::read_sram(uint16_t const address) const -> uint8_t {
  return sram_[address - 0x6000];
}

auto mapper_nrom::write_sram(uint16_t const address, uint8_t const data) -> void {
  sram_[address - 0x6000] = data;
}
