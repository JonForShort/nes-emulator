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
#ifndef JONES_PPU_FRAME_STATE_HH
#define JONES_PPU_FRAME_STATE_HH

#include "bit_utils.hh"

#include <cstdint>

namespace jones::ppu {

using ppu_frame_state_mask = uint32_t;

//
// States are from timing diagram: https://wiki.nesdev.com/w/images/4/4f/Ppu.svg
//
enum class ppu_frame_state : ppu_frame_state_mask {

  STATE_IDLE = jones::util::bit_value(0),

  STATE_FLAG_VBLANK_CLEAR = jones::util::bit_value(1),

  STATE_FLAG_VBLANK_SET = jones::util::bit_value(2),

  STATE_FLAG_VISIBLE = jones::util::bit_value(3),

  STATE_VRAM_FETCH_NT_BYTE = jones::util::bit_value(4),

  STATE_VRAM_FETCH_AT_BYTE = jones::util::bit_value(5),

  STATE_VRAM_FETCH_BG_LOW_BYTE = jones::util::bit_value(6),

  STATE_VRAM_FETCH_BG_HIGH_BYTE = jones::util::bit_value(7),

  STATE_VRAM_FETCH_SPRITE_BYTE = jones::util::bit_value(8),

  STATE_LOOPY_INC_HORI_V = jones::util::bit_value(9),

  STATE_LOOPY_INC_VERT_V = jones::util::bit_value(10),

  STATE_LOOPY_SET_HORI_V = jones::util::bit_value(11),

  STATE_LOOPY_SET_VERT_V = jones::util::bit_value(12),

  STATE_SECONDARY_OAM_CLEAR = jones::util::bit_value(13),

  STATE_EVALUATE_SPRITE = jones::util::bit_value(14),

  STATE_REG_BG_SHIFT = jones::util::bit_value(15),

  STATE_REG_BG_RELOAD = jones::util::bit_value(16),

  STATE_REG_SPRITE_SHIFT = jones::util::bit_value(17),
};

constexpr inline auto is_frame_state_set(ppu_frame_state_mask const state, ppu_frame_state const frame_state) {
  return (static_cast<uint32_t>(frame_state) & state) != 0;
}

} // namespace jones::ppu

#endif // JONES_PPU_FRAME_STATE_HH
