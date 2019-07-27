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

#include "mapper_cnrom.hh"

using namespace jones;

mapper_cnrom::mapper_cnrom(mapper_view const &mapper_view)
    : mapper(mapper_view),
      prgrom_(get_cartridge().address() + get_cartridge().header()->prgrom_offset()),
      prgrom_size_(get_cartridge().header()->prgrom_size()),
      sram_(0x2000, 0) {
  initialize_chrrom();
  mapper_view.cpu_memory().map(std::make_unique<memory_mappable_component<mapper_cnrom>>(this, "mapper_cnrom", 0x6000, 0xFFFF));
  mapper_view.ppu_memory().map(std::make_unique<memory_mappable_component<mapper_cnrom>>(this, "mapper_cnrom", 0x0000, 0x1FFF));
}

auto mapper_cnrom::initialize_chrrom() -> void {
  if (auto const &header = get_cartridge().header();
      header->chrrom_size() == 0) {
    chrom_.assign(8192, 0);
  } else {
    chrom_.resize(header->chrrom_size());
    auto const chrrom_begin = get_cartridge().address() + header->chrrom_offset();
    auto const chrrom_end = chrrom_begin + header->chrrom_size();
    std::copy(chrrom_begin, chrrom_end, chrom_.begin());
  }
}

auto mapper_cnrom::peek(uint16_t const address) const -> uint8_t {
  return read(address);
}

auto mapper_cnrom::read(uint16_t const address) const -> uint8_t {
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

auto mapper_cnrom::write(uint16_t const address, uint8_t const data) -> void {
  if (address < 0x2000) {
    return write_chr(address, data);
  } else if (address >= 0x8000) {
    return write_chr_bank(address, data);
  } else if (address >= 0x6000) {
    return write_sram(address, data);
  }
  BOOST_STATIC_ASSERT("unexpected write for mapper nrom");
}

auto mapper_cnrom::read_prg(uint16_t const address) const -> uint8_t {
  if (address >= 0xC000) {
    auto const prg_bank = (prgrom_size_ / 0x4000) - 1;
    auto const offset = address - 0xC000;
    return prgrom_[(prg_bank * 0x4000) + offset];
  } else if (address >= 0x8000) {
    return prgrom_[address - 0x8000];
  }
  return 0;
}

auto mapper_cnrom::read_chr(uint16_t const address) const -> uint8_t {
  return chrom_[address];
}

auto mapper_cnrom::write_chr(uint16_t const address, uint8_t const data) -> void {
  auto const offset = chr_bank_ * 0x2000 + address;
  chrom_[offset] = data;
}

auto mapper_cnrom::write_chr_bank(uint16_t const address, uint8_t const data) -> void {
  boost::ignore_unused(address);
  chr_bank_ = static_cast<uint32_t>(data & 0x3U);
}

auto mapper_cnrom::read_sram(uint16_t const address) const -> uint8_t {
  return sram_[address - 0x6000];
}

auto mapper_cnrom::write_sram(uint16_t const address, uint8_t const data) -> void {
  sram_[address - 0x6000] = data;
}
