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
#ifndef JONES_PPU_PPU_FRAME_HH
#define JONES_PPU_PPU_FRAME_HH

#include <array>
#include <cstdint>

namespace jones::ppu {

class ppu_frame final {
public:
  ppu_frame();

  ~ppu_frame();

  auto initialize() -> void;

  auto tick() -> void;

private:
  auto initialize_prerender_scanline() -> void;

  auto initialize_postrender_scanline() -> void;

  auto initialize_visible_scanline() -> void;

  auto initialize_vblank_scanline() -> void;

private:
  uint16_t current_cycle_{};

  uint16_t current_scanline_{};

  static constexpr auto ppu_num_scanlines = 262;

  std::array<uint32_t, ppu_num_scanlines> scanlines_{};
};

} // namespace jones::ppu

#endif // JONES_PPU_PPU_FRAME_HH
