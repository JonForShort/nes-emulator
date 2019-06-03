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
#include "ppu_frame_state.hh"

#include <vector>

//
// PPU implementation is largely influenced by the following.
//
// * https://wiki.nesdev.com/w/index.php/PPU_rendering
//
// * https://wiki.nesdev.com/w/images/4/4f/Ppu.svg
//

using namespace jones::ppu;

namespace {

constexpr auto ppu_max_cycles = 340;

constexpr auto ppu_screen_width = 256;

constexpr auto ppu_screen_height = 240;

constexpr auto ppu_num_scanlines = 262;

constexpr auto ppu_num_cycles = 341;

constexpr auto ppu_oam_size = 32;

constexpr auto ppu_palette_size = 32;

constexpr auto ppu_tile_width = 8;

constexpr auto ppu_tile_height = 8;

constexpr auto ppu_scanline_prerender = 261;

constexpr auto ppu_scanline_visible_start = 0;

constexpr auto ppu_scanline_visible_end = 239;

constexpr auto ppu_scanline_postrender = 240;

constexpr auto ppu_scanline_vblank = 241;

} // namespace

ppu_frame::ppu_frame(const mask_register &mask_register) : mask_register_(mask_register) {
  const auto scanline_cycles = std::vector<ppu_frame_state_mask>(ppu_num_cycles, 0);
  scanlines_ = std::vector<ppu_frame_cycles>(ppu_num_scanlines, scanline_cycles);

  buffer_.resize(ppu_screen_height);
  for (auto &height : buffer_) {
    height.resize(ppu_screen_width);
  }
}

auto ppu_frame::initialize() -> void {
  initialize_prerender_scanline();
  initialize_postrender_scanline();
  initialize_visible_scanline();
  initialize_vblank_scanline();
}

auto ppu_frame::step() -> uint16_t {
  process_frame_state();
  update_frame_counters();
  return 1;
}

auto ppu_frame::process_frame_state() -> void {
  const auto state = current_frame_state();
  if (is_frame_state_set(state, ppu_frame_state::STATE_FLAG_VISIBLE)) {
    process_state_flag_visible();
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_REG_BG_SHIFT)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_REG_SPRITE_SHIFT)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_REG_BG_RELOAD)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE)) {
    process_state_vram_fetch_nt_byte();
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_FLAG_VBLANK_SET)) {
    process_state_flag_vblank_set();
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_FLAG_VBLANK_CLEAR)) {
    process_state_flag_vblank_clear();
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_INC_HORI_V)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_INC_VERT_V)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_SET_HORI_V)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_SET_VERT_V)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_SECONDARY_OAM_CLEAR)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_EVALUATE_SPRITE)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_SPRITE_LOW_BYTE)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_SPRITE_HIGH_BYTE)) {
  }
  if (is_frame_state_set(state, ppu_frame_state::STATE_IDLE)) {
    //
    // Nothing to do.
    //
  }
}

auto ppu_frame::process_state_vram_fetch_nt_byte() -> void {
  const auto is_background_enabled = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
  if (!is_background_enabled) {
    return;
  }
}

auto ppu_frame::process_state_flag_vblank_set() -> void {
  buffer_ready_ = true;
}

auto ppu_frame::process_state_flag_vblank_clear() -> void {
  buffer_ready_ = false;
}

auto ppu_frame::process_state_flag_visible() -> void {
  const auto screen_x_position = current_cycle_ - 2;
  const auto screen_y_position = current_scanline_;

  const auto is_background_clipped = mask_register_.is_set(mask_flag::SHOW_LEFT_BACKGROUND) && screen_x_position < ppu_tile_width;
  const auto is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
  if (is_background_visible && !is_background_clipped) {
  }

  const auto is_sprite_clipped = mask_register_.is_set(mask_flag::SHOW_LEFT_SPRITES) && screen_x_position < ppu_tile_width;
  const auto is_sprite_visible = mask_register_.is_set(mask_flag::SHOW_SPRITES);
  if (is_sprite_visible && !is_sprite_clipped) {
  }

  buffer_[screen_y_position][screen_x_position] = 0xFF000000U;
}

auto ppu_frame::update_frame_counters() -> void {
  current_cycle_ += 1;
  if (current_cycle_ <= ppu_max_cycles) {
    return;
  }
  current_cycle_ = 0;

  current_scanline_ += 1;
  if (current_scanline_ < ppu_num_scanlines) {
    return;
  }
  current_scanline_ = 0;

  current_frame_ += 1;

  const auto is_odd_frame = current_frame_ % 2 == 1;
  const auto is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
  if (is_odd_frame && is_background_visible) {
    current_cycle_ += 1;
  }
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

inline auto ppu_frame::current_frame_state() -> ppu_frame_state_mask {
  return scanlines_[current_scanline_][current_cycle_];
}