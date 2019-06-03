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
#include "memory.hh"
#include "ppu_frame_state.hh"
#include "ppu_render_context.hh"

#include <cstdint>
#include <vector>

namespace jones::ppu {

using ppu_frame_cycles = std::vector<ppu_frame_state_mask>;

using ppu_frame_scanlines = std::vector<ppu_frame_cycles>;

using ppu_frame_buffer = std::vector<std::vector<uint32_t>>;

class ppu_frame final {
public:
  ppu_frame(const memory &memory, const mask_register &mask_register);

  ~ppu_frame() = default;

  auto initialize() -> void;

  auto step() -> uint16_t;

  auto cycle() const -> auto {
    return current_cycle_;
  }

  auto scanline() const -> auto {
    return current_scanline_;
  }

  auto frame() const -> auto {
    return current_frame_;
  }

  auto buffer() const -> auto {
    return buffer_;
  }

  auto buffer_ready() const -> auto {
    return buffer_ready_;
  }

private:
  auto initialize_prerender_scanline() -> void;

  auto initialize_postrender_scanline() -> void;

  auto initialize_visible_scanline() -> void;

  auto initialize_vblank_scanline() -> void;

  auto update_frame_counters() -> void;

  auto process_frame_state() -> void;

  auto current_frame_state() -> ppu_frame_state_mask;

  auto process_state_flag_visible() -> void;

  auto process_state_flag_vblank_set() -> void;

  auto process_state_flag_vblank_clear() -> void;

  auto process_state_vram_fetch_nt_byte() -> void;

private:
  uint16_t current_cycle_{};

  uint16_t current_scanline_{};

  uint64_t current_frame_{};

  ppu_frame_scanlines scanlines_{};

  ppu_frame_buffer buffer_{};

  ppu_render_context render_context_{};

  bool buffer_ready_{};

  const memory &memory_;

  const mask_register &mask_register_;
};

} // namespace jones::ppu

#endif // JONES_PPU_PPU_FRAME_HH
