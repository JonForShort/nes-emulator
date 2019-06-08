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
#ifndef JONES_PPU_IO_CONTEXT_HH
#define JONES_PPU_IO_CONTEXT_HH

#include <cstdint>

namespace jones::ppu {

union ppu_vram_address {

  uint16_t value : 15;

  struct {

    uint16_t coarse_x_scroll : 5;

    uint16_t coarse_y_scroll : 5;

    uint16_t h_nametable : 1;

    uint16_t v_nametable : 1;

    uint16_t fine_y_scroll : 3;
  };
};

union ppu_attribute_address {

  uint16_t value : 12;

  struct {

    uint16_t high_coarse_x : 3;

    uint16_t high_coarse_y : 3;

    uint16_t attribute_offset : 4;

    uint16_t h_name_table : 1;

    uint16_t v_name_table : 1;
  };
};

union ppu_attribute {

  uint8_t value;

  struct {

    uint8_t top_left : 2;

    uint8_t top_right : 2;

    uint8_t bottom_left : 2;

    uint8_t bottom_right : 2;
  };
};

struct ppu_io_context {

  bool vram_address_latch{};

  ppu_vram_address vram_address{};

  ppu_vram_address vram_address_temporary{};

  uint8_t fine_x_scroll{};
};

} // namespace jones::ppu

#endif // JONES_PPU_IO_CONTEXT_HH
