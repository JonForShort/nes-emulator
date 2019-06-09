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
#ifndef JONES_PPU_RENDER_CONTEXT_HH
#define JONES_PPU_RENDER_CONTEXT_HH

#include <cstdint>

namespace jones::ppu {

//
// https://wiki.nesdev.com/w/index.php/PPU_rendering
//
struct ppu_render_context {

  uint8_t name_table{};

  uint8_t attribute_table{};

  uint8_t attribute_table_latch{};

  uint8_t attribute_table_shift_low{};

  uint8_t attribute_table_shift_high{};

  uint8_t background_low{};

  uint8_t background_high{};

  uint16_t background_shift_low{};

  uint16_t background_shift_high{};

  uint8_t sprite_shift_low[8]{};

  uint8_t sprite_shift_high[8]{};

  uint8_t sprite_attribute_latches[8]{};

  uint8_t sprite_x_position_counters[8]{};

  bool sprite_zero_fetched{};
};

} // namespace jones::ppu

#endif // JONES_PPU_RENDER_CONTEXT_HH
