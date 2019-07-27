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
#include <boost/static_assert.hpp>

#include "configuration.hh"
#include "mapper_mmc1.hh"

using namespace jones;

namespace jc = jones::configuration;

mapper_mmc1::mapper_mmc1(mapper_view const &mapper_view)
    : mapper(mapper_view), shift_register_(0x10) {
  const auto &cartridge = mapper_view.cartridge();

  prg_rom_size_ = cartridge.header()->prgrom_size();
  if (prg_rom_size_ > 0) {
    prg_rom_ = cartridge.address() + cartridge.header()->prgrom_offset();
  }

  chr_rom_size_ = cartridge.header()->chrrom_size();
  if (chr_rom_size_ > 0) {
    chr_rom_ = cartridge.address() + cartridge.header()->chrrom_offset();
  } else {
    chr_ram_.resize(8192);
    chr_rom_size_ = 8192;
    chr_rom_ = &chr_ram_[0];
  }

  sram_.resize(0x2000);
  std::fill(sram_.begin(), sram_.end(), 0);

  prg_offsets_[1] = get_prg_bank_offset(-1);

  mapper_view.cpu_memory().map(std::make_unique<memory_mappable_component<mapper_mmc1>>(this, "mapper_mmc1", 0x6000, 0xFFFF));
  mapper_view.ppu_memory().map(std::make_unique<memory_mappable_component<mapper_mmc1>>(this, "mapper_mmc1", 0x0000, 0x1FFF));
}

auto mapper_mmc1::get_prg_bank_offset(int const index) const -> uint16_t {
  auto adjusted_index = index;
  if (adjusted_index >= 0x80) {
    adjusted_index -= 0x100;
  }
  adjusted_index %= prg_rom_size_ / 0x4000;
  auto offset = adjusted_index * 0x4000;
  if (offset < 0) {
    offset += prg_rom_size_;
  }
  return offset;
}

auto mapper_mmc1::get_chr_bank_offset(int const index) const -> uint16_t {
  auto adjusted_index = index;
  if (adjusted_index >= 0x80) {
    adjusted_index -= 0x100;
  }
  adjusted_index %= chr_rom_size_ / 0x1000;
  auto offset = adjusted_index * 0x1000;
  if (offset < 0) {
    offset += chr_rom_size_;
  }
  return offset;
}

auto mapper_mmc1::update_offsets() -> void {
  switch (prg_mode_) {
  case 0:
  case 1: {
    prg_offsets_[0] = get_prg_bank_offset(prg_bank_ & 0xFEU);
    prg_offsets_[1] = get_prg_bank_offset(prg_bank_ | 0x01U);
    break;
  }
  case 2: {
    prg_offsets_[0] = 0;
    prg_offsets_[1] = get_prg_bank_offset(prg_bank_);
    break;
  }
  case 3: {
    prg_offsets_[0] = get_prg_bank_offset(prg_bank_);
    prg_offsets_[1] = get_prg_bank_offset(-1);
    break;
  }
  }
  switch (chr_mode_) {
  case 0: {
    chr_offsets_[0] = get_chr_bank_offset(chr_bank_[0] & 0xFEU);
    chr_offsets_[1] = get_chr_bank_offset(chr_bank_[0] | 0x01U);
    break;
  }
  case 1: {
    chr_offsets_[0] = get_chr_bank_offset(chr_bank_[0]);
    chr_offsets_[1] = get_chr_bank_offset(chr_bank_[1]);
    break;
  }
  }
}

auto mapper_mmc1::peek(uint16_t const address) const -> uint8_t {
  return read(address);
}

auto mapper_mmc1::read(uint16_t const address) const -> uint8_t {
  if (address < 0x2000) {
    const auto bank = address / 0x1000;
    const auto offset = address % 0x1000;
    const auto chr_offset = chr_offsets_[bank] + offset;
    return chr_rom_[chr_offset];
  }
  if (address >= 0x8000) {
    const auto adjusted_address = address - 0x8000;
    const auto bank = adjusted_address / 0x4000;
    const auto offset = adjusted_address % 0x4000;
    const auto prg_offset = prg_offsets_[bank] + offset;
    return prg_rom_[prg_offset];
  }
  if (address >= 0x6000) {
    const auto adjusted_address = address - 0x6000;
    return sram_[adjusted_address];
  }
  BOOST_STATIC_ASSERT("unexpected read for mmc1 mapper");
  return 0;
}

auto mapper_mmc1::write(uint16_t const address, uint8_t const data) -> void {
  if (address < 0x2000) {
    const auto bank = address / 0x1000;
    const auto offset = address % 0x1000;
    const auto chr_offset = chr_offsets_[bank] + offset;
    chr_rom_[chr_offset] = data;
  }
  if (address >= 0x8000) {
    load_register(address, data);
  }
  if (address >= 0x6000) {
    const auto adjusted_address = address - 0x6000;
    sram_[adjusted_address] = data;
  }
  BOOST_STATIC_ASSERT("unexpected read for mmc1 mapper");
}

auto mapper_mmc1::load_register(uint16_t const address, uint8_t const data) -> void {
  if ((data & 0x80U) == 0x80U) {
    shift_register_ = 0x10U;
    write_control_register(control_register_ | 0x0CU);
  } else {
    const auto is_register_complete = (shift_register_ & 1) == 1;
    shift_register_ >>= 1U;
    shift_register_ |= (data & 1U) << 4;
    if (is_register_complete) {
      write_register(address, shift_register_);
      shift_register_ = 0x10;
    }
  }
}

auto mapper_mmc1::write_register(uint16_t const address, uint8_t const data) -> void {
  if (address <= 0x9FFF) {
    write_control_register(data);
  } else if (address <= 0xBFFF) {
    write_chr_bank_zero(data);
  } else if (address <= 0xDFFF) {
    write_chr_bank_one(data);
  } else {
    write_prg_bank(data);
  }
}

auto mapper_mmc1::write_chr_bank_zero(const uint8_t data) -> void {
  chr_bank_[0] = data;
  update_offsets();
}

auto mapper_mmc1::write_chr_bank_one(const uint8_t data) -> void {
  chr_bank_[1] = data;
  update_offsets();
}

auto mapper_mmc1::write_prg_bank(const uint8_t data) -> void {
  prg_bank_ = data & 0x0FU;
}

auto mapper_mmc1::write_control_register(const uint8_t data) -> void {
  control_register_ = data;
  chr_mode_ = (data >> 4U) & 1U;
  prg_mode_ = (data >> 2U) & 3U;

  const auto mirror_mode = jc::get_mirror_mode(data & 3U);
  jc::configuration::instance().set<uint8_t>(jc::property::PROPERTY_MIRROR_MODE, mirror_mode);

  update_offsets();
}
