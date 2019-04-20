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

namespace {

const size_t chrram_size = 0x2000;

}

mapper_nrom::mapper_nrom(const jones::mapped_cartridge &cartridge)
    : mapper(cartridge),
      type_(resolve_type(cartridge)),
      mirroring_type_(resolve_mirroring_type(cartridge)),
      use_chrram(cartridge.header()->chrrom_size() <= 0) {
  if (use_chrram) {
    chrram_.resize(chrram_size);
    std::fill(chrram_.begin(), chrram_.end(), 0);
  }
}

uint8_t mapper_nrom::read_prg(uint16_t address) const {
  const auto base_prgrom_address = get_cartridge().address() + get_cartridge().header()->prgrom_offset();
  switch (type_) {
  case nrom_type::NROM_128: {
    return *(base_prgrom_address + (address & 0x3FFFU));
  }
  case nrom_type::NROM_256: {
    return *(base_prgrom_address + address);
  }
  default:
    break;
  }
  BOOST_STATIC_ASSERT("unexpected nrom type");
  return 0;
}

void mapper_nrom::write_prg(uint16_t address, uint8_t data) {
  boost::ignore_unused(address, data);
  BOOST_STATIC_ASSERT("unable to write to prg rom");
}

uint8_t mapper_nrom::read_chr(uint16_t address) const {
  if (use_chrram) {
    return chrram_[address];
  } else {
    return *(get_cartridge().address() + get_cartridge().header()->chrrom_offset());
  }
}

void mapper_nrom::write_chr(uint16_t address, uint8_t data) {
  if (use_chrram) {
    chrram_[address] = data;
  } else {
    BOOST_STATIC_ASSERT("unable to write to chr rom");
  }
}