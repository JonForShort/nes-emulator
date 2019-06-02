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

#include "mask_register.hh"

#include <cstdint>
#include <vector>

namespace jones::ppu {

using ppu_frame_state_mask = uint32_t;

using ppu_frame_cycles = std::vector<ppu_frame_state_mask>;

using ppu_frame_scanlines = std::vector<ppu_frame_cycles>;

class ppu_frame final {
public:
  explicit ppu_frame(const mask_register &mask_register);

  ~ppu_frame() = default;

  auto initialize() -> void;

  auto step() -> uint16_t;

  auto cycle() -> auto {
    return current_cycle_;
  }

  auto scanline() -> auto {
    return current_scanline_;
  }

  auto frame() -> auto {
    return current_frame_;
  }

private:
  auto initialize_prerender_scanline() -> void;

  auto initialize_postrender_scanline() -> void;

  auto initialize_visible_scanline() -> void;

  auto initialize_vblank_scanline() -> void;

  auto update_frame_counters() -> void;

  auto process_frame_state() -> void;

  auto current_frame_state() -> ppu_frame_state_mask;

private:
  uint16_t current_cycle_{};

  uint16_t current_scanline_{};

  uint64_t current_frame_{};

  ppu_frame_scanlines scanlines_{};

  const mask_register &mask_register_;
};

} // namespace jones::ppu

#endif // JONES_PPU_PPU_FRAME_HH
