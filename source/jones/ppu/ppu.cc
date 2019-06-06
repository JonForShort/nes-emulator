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
#include <boost/assert.hpp>
#include <boost/core/ignore_unused.hpp>
#include <boost/static_assert.hpp>

#include "control_register.hh"
#include "mask_register.hh"
#include "memory.hh"
#include "name_table.hh"
#include "palette.hh"
#include "pattern_table.hh"
#include "ppu.hh"
#include "ppu_frame_state.hh"
#include "ppu_render_context.hh"
#include "status_register.hh"

//
// PPU implementation is largely influenced by the following documents.
//
// https://wiki.nesdev.com/w/index.php/PPU_rendering
//
// https://wiki.nesdev.com/w/images/4/4f/Ppu.svg
//

using namespace jones::ppu;

using ppu_frame_cycles = std::vector<ppu_frame_state_mask>;

using ppu_frame_scanlines = std::vector<ppu_frame_cycles>;

using ppu_frame_buffer = std::vector<std::vector<uint32_t>>;

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

template <typename T>
using memory_mappable_component = jones::memory_mappable_component<T>;

class ppu::impl final {
public:
  impl(memory &cpu_memory, memory &ppu_memory)
      : cpu_memory_(cpu_memory), ppu_memory_(ppu_memory) {
    ppu_memory_.map(std::make_unique<memory_mappable_component<pattern_table>>(&pattern_table_, pattern_table_memory_begin, pattern_table_memory_end));
    ppu_memory_.map(std::make_unique<memory_mappable_component<name_table>>(&name_table_, name_table_memory_begin, name_table_memory_end));
    ppu_memory_.map(std::make_unique<memory_mappable_component<palette>>(&palette_, palette_memory_begin, palette_memory_end));
  }

  auto initialize() -> void {
    initialize_frame_scanlines();
    initialize_prerender_scanline();
    initialize_postrender_scanline();
    initialize_visible_scanline();
    initialize_vblank_scanline();
  }

  auto uninitialize() -> void {
    //
    // nothing to do.
    //
  }

  auto read(const uint16_t address) const -> uint8_t {
    if (address >= 0x2000 && address <= 0x3FFF) {
      return read_registers(address);
    } else if (address == 0x4014) {
      return read_object_attribute_memory_dma();
    } else {
      BOOST_STATIC_ASSERT("read unexpected for ppu");
    }
    return -1;
  }

  auto write(const uint16_t address, const uint8_t data) -> void {
    if (address >= 0x2000 && address <= 0x3FFF) {
      write_registers(address, data);
    } else if (address == 0x4014) {
      write_object_attribute_memory_dma(data);
    } else {
      BOOST_STATIC_ASSERT("write unexpected for ppu");
    }
  }

  auto step() -> uint8_t {
    process_frame_state();
    update_frame_counters();
    return 1;
  }

  auto get_state() -> ppu_state {
    return ppu_state{
        .cycle = frame_current_cycle_,
        .scanline = frame_current_scanline_,
        .frame = frame_current_frame_};
  }

  auto get_buffer() const -> auto {
    return frame_buffer_;
  }

  auto is_buffer_ready() const -> auto {
    return is_buffer_ready_;
  }

private:
  auto write_registers(const uint16_t address, const uint8_t data) -> void {
    BOOST_ASSERT_MSG(address >= 0x2000 && address <= 0x3FFF, "write unexpected address for ppu");
    status_register_.register_updated(data);
    const auto address_offset = (address - 0x2000);
    switch (address_offset % 8) {
    case 0:
      write_control(data);
      break;
    case 1:
      write_mask(data);
      break;
    case 2:
      write_status(data);
      break;
    case 3:
      write_object_attribute_memory_address(data);
      break;
    case 4:
      write_object_attribute_memory_data(data);
      break;
    case 5:
      write_scroll(data);
      break;
    case 6:
      write_address(data);
      break;
    case 7:
      write_data(data);
      break;
    }
  }

  auto write_control(const uint8_t data) -> void {
    control_register_.set(data);
  }

  auto write_mask(const uint8_t data) -> void {
    mask_register_.set(data);
  }

  auto write_status(const uint8_t data) -> void {
    BOOST_STATIC_ASSERT("write unexpected for status");
    status_register_.set(data);
  }

  auto write_object_attribute_memory_address(const uint8_t data) -> void {
    oam_address_ = data;
  }

  auto write_object_attribute_memory_data(const uint8_t data) -> void {
    oam_data_[oam_address_] = data;
    oam_address_ += 1;
  }

  auto write_scroll(const uint8_t data) -> void {
    boost::ignore_unused(data);
  }

  auto write_address(const uint8_t data) -> void {
    boost::ignore_unused(data);
  }

  auto write_data(const uint8_t data) -> void {
    boost::ignore_unused(data);
  }

  auto write_object_attribute_memory_dma(const uint8_t data) -> void {
    boost::ignore_unused(data);
  }

  auto read_registers(const uint16_t address) const -> uint8_t {
    BOOST_ASSERT_MSG(address >= 0x2000 && address <= 0x3FFF, "read unexpected address for ppu");
    const auto address_offset = (address - 0x2000);
    switch (address_offset % 8) {
    case 0:
      return read_control();
    case 1:
      return read_mask();
    case 2:
      return read_status();
    case 3:
      return read_object_attribute_memory_address();
    case 4:
      return read_object_attribute_memory_data();
    case 5:
      return read_scroll();
    case 6:
      return read_address();
    case 7:
      return read_data();
    }
    return -1;
  }

  auto read_control() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for control");
    return control_register_.get();
  }

  auto read_mask() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for mask");
    return mask_register_.get();
  }

  auto read_status() const -> uint8_t {
    return status_register_.get();
  }

  auto read_object_attribute_memory_address() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for oam address");
    return oam_address_;
  }

  auto read_object_attribute_memory_data() const -> uint8_t {
    return oam_data_[oam_address_];
  }

  auto read_scroll() const -> uint8_t {
    return 0;
  }

  auto read_address() const -> uint8_t {
    return 0;
  }

  auto read_data() const -> uint8_t {
    return 0;
  }

  auto read_object_attribute_memory_dma() const -> uint8_t {
    return 0;
  }

  auto process_frame_state() -> void {
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

  auto process_state_vram_fetch_nt_byte() -> void {
    const auto is_background_enabled = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_enabled) {
      return;
    }
  }

  auto process_state_flag_vblank_set() -> void {
    is_buffer_ready_ = true;
  }

  auto process_state_flag_vblank_clear() -> void {
    is_buffer_ready_ = false;
  }

  auto process_state_flag_visible() -> void {
    const auto screen_x_position = frame_current_cycle_ - 2;
    const auto screen_y_position = frame_current_scanline_;

    const auto is_background_clipped = mask_register_.is_set(mask_flag::SHOW_LEFT_BACKGROUND) && screen_x_position < ppu_tile_width;
    const auto is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (is_background_visible && !is_background_clipped) {
    }

    const auto is_sprite_clipped = mask_register_.is_set(mask_flag::SHOW_LEFT_SPRITES) && screen_x_position < ppu_tile_width;
    const auto is_sprite_visible = mask_register_.is_set(mask_flag::SHOW_SPRITES);
    if (is_sprite_visible && !is_sprite_clipped) {
    }

    frame_buffer_[screen_y_position][screen_x_position] = 0xFF000000U;
  }

  auto update_frame_counters() -> void {
    frame_current_cycle_ += 1;
    if (frame_current_cycle_ <= ppu_max_cycles) {
      return;
    }
    frame_current_cycle_ = 0;

    frame_current_scanline_ += 1;
    if (frame_current_scanline_ < ppu_num_scanlines) {
      return;
    }
    frame_current_scanline_ = 0;

    frame_current_frame_ += 1;

    const auto is_odd_frame = frame_current_frame_ % 2 == 1;
    const auto is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (is_odd_frame && is_background_visible) {
      frame_current_cycle_ += 1;
    }
  }

  auto initialize_frame_scanlines() -> void {
    const auto scanline_cycles = std::vector<ppu_frame_state_mask>(ppu_num_cycles, 0);
    frame_scanlines_ = std::vector<ppu_frame_cycles>(ppu_num_scanlines, scanline_cycles);

    frame_buffer_.resize(ppu_screen_height);
    for (auto &height : frame_buffer_) {
      height.resize(ppu_screen_width);
    }
  }

  auto initialize_prerender_scanline() -> void {

    auto &scanline = frame_scanlines_[ppu_scanline_prerender];

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

  auto initialize_postrender_scanline() -> void {
    for (auto i = 0; i <= ppu_max_cycles; i++) {
      frame_scanlines_[ppu_scanline_postrender][i] |= static_cast<uint32_t>(ppu_frame_state::STATE_IDLE);
    }
    for (auto i = 242; i <= 260; i++) {
      auto &scanline = frame_scanlines_[i];
      for (auto j = 0; j <= ppu_max_cycles; j++) {
        scanline[j] |= static_cast<uint32_t>(ppu_frame_state::STATE_IDLE);
      }
    }
  }

  auto initialize_visible_scanline() -> void {

    for (auto j = ppu_scanline_visible_start; j <= ppu_scanline_visible_end; j++) {

      auto &scanline = frame_scanlines_[j];

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

  auto initialize_vblank_scanline() -> void {
    frame_scanlines_[ppu_scanline_vblank][1] |= static_cast<uint32_t>(ppu_frame_state::STATE_FLAG_VBLANK_SET);
  }

  inline auto current_frame_state() -> ppu_frame_state_mask {
    return frame_scanlines_[frame_current_scanline_][frame_current_cycle_];
  }

private:
  memory &cpu_memory_;

  memory &ppu_memory_;

  control_register control_register_{};

  mask_register mask_register_{};

  status_register status_register_{};

  uint8_t oam_address_{};

  uint8_t oam_data_[0xFF] = {0};

  palette palette_{};

  name_table name_table_{};

  pattern_table pattern_table_{};

  uint16_t frame_current_cycle_{};

  uint16_t frame_current_scanline_{};

  uint64_t frame_current_frame_{};

  ppu_frame_scanlines frame_scanlines_{};

  ppu_frame_buffer frame_buffer_{};

  ppu_render_context render_context_{};

  bool is_buffer_ready_{};
};

ppu::ppu(jones::memory &cpu_memory, jones::memory &ppu_memory)
    : impl_(new impl(cpu_memory, ppu_memory)) {
  //
  // nothing to do.
  //
}

ppu::~ppu() = default;

auto ppu::initialize() -> void {
  impl_->initialize();
}

auto ppu::uninitialize() -> void {
  impl_->uninitialize();
}

auto ppu::read(const uint16_t address) const -> uint8_t {
  return impl_->read(address);
}

auto ppu::write(const uint16_t address, const uint8_t data) -> void {
  impl_->write(address, data);
}

auto ppu::step() -> uint8_t {
  return impl_->step();
}

auto ppu::get_state() const -> ppu_state {
  return impl_->get_state();
}

auto ppu::get_buffer() const -> std::vector<std::vector<uint32_t>> {
  return impl_->get_buffer();
}

auto ppu::is_buffer_ready() const -> bool {
  return impl_->is_buffer_ready();
}
