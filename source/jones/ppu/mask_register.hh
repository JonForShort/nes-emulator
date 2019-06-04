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
#ifndef JONES_PPU_MASK_REGISTER_HH
#define JONES_PPU_MASK_REGISTER_HH

#include <bitset>
#include <cstdint>

namespace jones::ppu {

//
// https://wiki.nesdev.com/w/index.php/PPU_registers#PPUMASK
//
enum class mask_flag : uint16_t {

  USE_GRAYSCALE,

  SHOW_LEFT_BACKGROUND,

  SHOW_LEFT_SPRITES,

  SHOW_BACKGROUND,

  SHOW_SPRITES,

  TINT_RED,

  TINT_GREEN,

  TINT_BLUE,

  COUNT
};

class mask_register final {
public:
  auto is_set(mask_flag flag) const -> bool;

  auto clear(mask_flag flag) -> void;

  auto set(mask_flag flag) -> void;

  auto set(uint8_t flags) -> void;

  auto get() const -> uint8_t;

private:
  static constexpr auto mask_flag_count = static_cast<uint16_t>(mask_flag::COUNT);

  std::bitset<mask_flag_count> mask_flags_;
};

} // namespace jones::ppu

#endif // JONES_PPU_MASK_REGISTER_HH
