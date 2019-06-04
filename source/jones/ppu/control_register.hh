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
#ifndef JONES_PPU_CONTROL_REGISTER_HH
#define JONES_PPU_CONTROL_REGISTER_HH

#include <bitset>
#include <cstdint>

namespace jones::ppu {

//
// https://wiki.nesdev.com/w/index.php/PPU_registers#PPUCTRL
//
enum class control_flag : uint16_t {

  NAME_TABLE_ONE = 0,

  NAME_TABLE_TWO,

  VRAM_INCREMENT,

  SPRITE_TABLE,

  BACKGROUND_TABLE,

  SPRITE_SIZE,

  MASTER_SLAVE,

  NMI,

  COUNT
};

class control_register final {
public:
  auto is_set(control_flag flag) const -> bool;

  auto clear(control_flag flag) -> void;

  auto set(control_flag flag) -> void;

  auto set(uint8_t flags) -> void;

  auto get() const -> uint8_t;

private:
  static constexpr auto control_flag_count = static_cast<uint16_t>(control_flag::COUNT);

  std::bitset<control_flag_count> control_flags_;
};

} // namespace jones::ppu

#endif // JONES_PPU_CONTROL_REGISTER_HH
