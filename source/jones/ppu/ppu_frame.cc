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
// PPU implementation is largely influenced by the following documents.
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

constexpr auto ppu_frame_state_render = 0;

} // namespace

ppu_frame::ppu_frame() = default;

ppu_frame::~ppu_frame() = default;

auto ppu_frame::initialize() -> void {
  initialize_prerender_scanline();
  initialize_postrender_scanline();
  initialize_visible_scanline();
  initialize_vblank_scanline();
}

auto ppu_frame::tick() -> void {
}

auto ppu_frame::initialize_prerender_scanline() -> void {
}

auto ppu_frame::initialize_postrender_scanline() -> void {
}

auto ppu_frame::initialize_visible_scanline() -> void {
}

auto ppu_frame::initialize_vblank_scanline() -> void {
}