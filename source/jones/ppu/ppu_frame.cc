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
#include "ppu_frame.hh"

//
// PPU implementation is largely influenced by the following.
//
// * https://wiki.nesdev.com/w/index.php/PPU_rendering
//
// * https://wiki.nesdev.com/w/images/4/4f/Ppu.svg
//

using namespace jones::ppu;

namespace {

constexpr auto ppu_initial_cycles = 0;

constexpr auto ppu_max_cycles = 340;

constexpr auto ppu_screen_width = 256;

constexpr auto ppu_screen_height = 240;

constexpr auto ppu_screen_refresh_rate = 60.0988;

constexpr auto ppu_max_dots = 341;

constexpr auto ppu_num_scanlines = 262;

constexpr auto ppu_oam_size = 32;

constexpr auto ppu_palette_size = 32;

constexpr auto ppu_tile_width = 8;

constexpr auto ppu_tile_height = 8;

constexpr auto ppu_scanline_prerender = 261;

constexpr auto ppu_scanline_visible_start = 0;

constexpr auto ppu_scanline_visible_end = 239;

constexpr auto ppu_scanline_postrender = 240;

constexpr auto ppu_scanline_vblank = 241;

constexpr inline uint32_t bit_value(const uint8_t i) {
  return 1U << i;
}

//
// States are from timing diagram: https://wiki.nesdev.com/w/images/4/4f/Ppu.svg
//
enum class ppu_frame_state : uint32_t {

  STATE_IDLE = bit_value(0),

  STATE_FLAG_VBLANK_CLEAR = bit_value(1),

  STATE_FLAG_VBLANK_SET = bit_value(2),

  STATE_FLAG_VISIBLE = bit_value(3),

  STATE_VRAM_FETCH_NT_BYTE = bit_value(4),

  STATE_VRAM_FETCH_AT_BYTE = bit_value(5),

  STATE_VRAM_FETCH_BG_LOW_BYTE = bit_value(6),

  STATE_VRAM_FETCH_BG_HIGH_BYTE = bit_value(7),

  STATE_VRAM_FETCH_SPRITE_LOW_BYTE = bit_value(8),

  STATE_VRAM_FETCH_SPRITE_HIGH_BYTE = bit_value(9),

  STATE_LOOPY_INC_HORI_V = bit_value(10),

  STATE_LOOPY_INC_VERT_V = bit_value(11),

  STATE_LOOPY_SET_HORI_V = bit_value(12),

  STATE_LOOPY_SET_VERT_V = bit_value(13),

  STATE_SECONDARY_OAM_CLEAR = bit_value(14),

  STATE_EVALUATE_SPRITE = bit_value(15),

  STATE_REG_BG_SHIFT = bit_value(16),

  STATE_REG_BG_RELOAD = bit_value(17),

  STATE_REG_SPRITE_SHIFT = bit_value(18),
};

} // namespace

ppu_frame::ppu_frame() = default;

ppu_frame::~ppu_frame() = default;

auto ppu_frame::initialize() -> void {
  initialize_prerender_scanline();
  initialize_postrender_scanline();
  initialize_visible_scanline();
  initialize_vblank_scanline();
}

auto ppu_frame::step() -> void {
}

auto ppu_frame::initialize_prerender_scanline() -> void {

  auto &scanline = scanlines_[ppu_scanline_prerender];

  scanline[0] |= static_cast<uint32_t>(ppu_frame_state::STATE_IDLE);

  scanline[1] |= static_cast<uint32_t>(ppu_frame_state::STATE_FLAG_VBLANK_CLEAR);

  for (auto i = 1; i <= 256; i++) {
    if (i % 8 == 1) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
    }
    if (i % 8 == 3) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE);
    }
    if (i % 8 == 5) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE);
    }
    if (i % 8 == 7) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE);
    }
  }

  for (auto i = 2; i <= 257; i++) {
    scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_SHIFT);
    scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_SPRITE_SHIFT);
  }

  for (auto i = 9; i <= 257; i++) {
    if (i % 8 == 1) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_RELOAD);
    }
  }

  for (auto i = 8; i <= 256; i++) {
    if (i % 8 == 0) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_HORI_V);
    }
  }

  scanline[256] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_VERT_V);
  scanline[257] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_SET_HORI_V);

  for (auto i = 257; i < 320; i++) {
    if (i % 8 == 5) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_SPRITE_LOW_BYTE);
    }
    if (i % 8 == 7) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_SPRITE_HIGH_BYTE);
    }
  }

  for (auto i = 280; i <= 304; i++) {
    scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_SET_VERT_V);
  }

  for (auto i = 321; i <= 336; i += 8) {
    scanline[i + 0] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
    scanline[i + 2] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE);
    scanline[i + 4] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE);
    scanline[i + 6] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE);
  }

  for (auto i = 322; i <= 337; i++) {
    scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_SHIFT);
  }

  for (auto i = 329; i <= 337; i += 8) {
    scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_RELOAD);
  }

  for (auto i = 328; i <= 336; i += 8) {
    scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_HORI_V);
  }

  for (auto i = 337; i <= 340; i += 4) {
    scanline[i + 0] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
    scanline[i + 2] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
  }
}

auto ppu_frame::initialize_postrender_scanline() -> void {
  for (auto i = 0; i <= ppu_max_cycles; i++) {
    scanlines_[ppu_scanline_postrender][i] |= static_cast<uint32_t>(ppu_frame_state::STATE_IDLE);
  }
  for (auto i = 242; i <= 260; i++) {
    auto &scanline = scanlines_[i];
    for (auto j = 0; j <= ppu_max_cycles; j++) {
      scanline[j] |= static_cast<uint32_t>(ppu_frame_state::STATE_IDLE);
    }
  }
}

auto ppu_frame::initialize_visible_scanline() -> void {

  for (auto j = ppu_scanline_visible_start; j <= ppu_scanline_visible_end; j++) {

    auto &scanline = scanlines_[j];

    scanline[0] |= static_cast<uint32_t>(ppu_frame_state::STATE_IDLE);

    for (auto i = 2; i <= 257; i++) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_FLAG_VISIBLE);
    }

    for (auto i = 1; i <= 256; i++) {
      if (i % 8 == 1) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
      }
      if (i % 8 == 3) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE);
      }
      if (i % 8 == 5) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE);
      }
      if (i % 8 == 7) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE);
      }
    }

    for (auto i = 2; i <= 257; i++) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_SHIFT);
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_SPRITE_SHIFT);
    }

    for (auto i = 9; i <= 257; i++) {
      if (i % 8 == 1) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_RELOAD);
      }
    }

    for (auto i = 8; i <= 256; i++) {
      if (i % 8 == 0) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_HORI_V);
      }
    }

    scanline[256] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_VERT_V);
    scanline[257] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_SET_HORI_V);

    for (auto i = 257; i < 320; i++) {
      if (i % 8 == 5) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_SPRITE_LOW_BYTE);
      }
      if (i % 8 == 7) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_SPRITE_HIGH_BYTE);
      }
    }

    for (auto i = 321; i <= 336; i += 8) {
      scanline[i + 0] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
      scanline[i + 2] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE);
      scanline[i + 4] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE);
      scanline[i + 6] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE);
    }

    for (auto i = 322; i <= 337; i++) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_SHIFT);
    }

    for (auto i = 329; i <= 337; i += 8) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_RELOAD);
    }

    for (auto i = 328; i <= 336; i += 8) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_HORI_V);
    }

    for (auto i = 337; i <= 340; i += 4) {
      scanline[i + 0] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
      scanline[i + 2] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
    }

    scanline[1] |= static_cast<uint32_t>(ppu_frame_state::STATE_SECONDARY_OAM_CLEAR);

    scanline[65] |= static_cast<uint32_t>(ppu_frame_state::STATE_EVALUATE_SPRITE);
  }
}

auto ppu_frame::initialize_vblank_scanline() -> void {
  scanlines_[ppu_scanline_vblank][1] |= static_cast<uint32_t>(ppu_frame_state::STATE_FLAG_VBLANK_SET);
}