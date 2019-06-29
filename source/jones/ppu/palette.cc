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
#include "palette.hh"

#include <boost/core/ignore_unused.hpp>
#include <boost/static_assert.hpp>

using namespace jones::ppu;

namespace {

//
// https://wiki.nesdev.com/w/index.php/PPU_palettes
//
constexpr std::uint32_t ppu_palette[] = {
    0x666666ff,
    0x002a88ff,
    0x1412a7ff,
    0x3b00a4ff,
    0x5c007eff,
    0x6e0040ff,
    0x6c0600ff,
    0x561d00ff,
    0x333500ff,
    0x0b4800ff,
    0x005200ff,
    0x004f08ff,
    0x00404dff,
    0x000000ff,
    0x000000ff,
    0x000000ff,
    0xadadadff,
    0x155fd9ff,
    0x4240ffff,
    0x7527feff,
    0xa01accff,
    0xb71e7bff,
    0xb53120ff,
    0x994e00ff,
    0x6b6d00ff,
    0x388700ff,
    0x0c9300ff,
    0x008f32ff,
    0x007c8dff,
    0x000000ff,
    0x000000ff,
    0x000000ff,
    0xfffeffff,
    0x64b0ffff,
    0x9290ffff,
    0xc676ffff,
    0xf36affff,
    0xfe6eccff,
    0xfe8170ff,
    0xea9e22ff,
    0xbcbe00ff,
    0x88d800ff,
    0x5ce430ff,
    0x45e082ff,
    0x48cddeff,
    0x4f4f4fff,
    0x000000ff,
    0x000000ff,
    0xfffeffff,
    0xc0dfffff,
    0xd3d2ffff,
    0xe8c8ffff,
    0xfbc2ffff,
    0xfec4eaff,
    0xfeccc5ff,
    0xf7d8a5ff,
    0xe4e594ff,
    0xcfef96ff,
    0xbdf4abff,
    0xb3f3ccff,
    0xb5ebf2ff,
    0xb8b8b8ff,
    0x000000ff,
    0x000000ff,
};

} // namespace

auto palette::peek(uint16_t const address) const -> uint8_t {
  return read(address);
}

auto palette::read(uint16_t const address) const -> uint8_t {
  const uint16_t address_offset = (address - palette_memory_begin);
  const uint16_t address_byte = address_offset % sizeof(uint32_t);
  const uint32_t palette = ppu_palette[address_offset / sizeof(uint32_t)];
  const uint8_t palette_byte = (palette >> (address_byte * 8)) & 0xFF;
  return palette_byte;
}

auto palette::write(uint16_t const address, uint8_t const data) -> void {
  boost::ignore_unused(address, data);
  BOOST_STATIC_ASSERT("unexpected write on palette");
}
