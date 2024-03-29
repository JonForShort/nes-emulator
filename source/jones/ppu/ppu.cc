//
// MIT License
//
// Copyright 2019-2021
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
#include <array>
#include <boost/assert.hpp>
#include <boost/static_assert.hpp>

#include "bit_utils.hh"
#include "control_register.hh"
#include "cpu.hh"
#include "mask_register.hh"
#include "memory.hh"
#include "name_table.hh"
#include "palette.hh"
#include "pattern_table.hh"
#include "ppu.hh"
#include "ppu_frame_state.hh"
#include "ppu_io_context.hh"
#include "ppu_render_context.hh"
#include "screen.hh"
#include "status_register.hh"

//
// PPU implementation is largely influenced by the following documents.
//
// https://wiki.nesdev.com/w/index.php/PPU_rendering
//
// https://wiki.nesdev.com/w/images/4/4f/Ppu.svg
//

using namespace jones::ppu;
using namespace jones::screen;
using namespace jones::util;

using ppu_frame_cycles = std::vector<ppu_frame_state_mask>;

using ppu_frame_scanlines = std::vector<ppu_frame_cycles>;

using ppu_frame_buffer = std::vector<std::vector<uint32_t>>;

namespace {

constexpr auto ppu_max_cycles = 340;

constexpr auto ppu_dma_cycles = 513;

constexpr auto ppu_max_sprites = 64;

constexpr auto ppu_max_scanlines = 262;

constexpr auto ppu_num_cycles = 341;

constexpr auto ppu_oam_size = 256;

constexpr auto ppu_oam_secondary_size = 32;

constexpr auto ppu_tile_width = 8;

constexpr auto ppu_tile_height = 8;

constexpr auto ppu_tile_size = 16;

constexpr auto ppu_sprites_per_line = 8;

constexpr auto ppu_scanline_prerender = 261;

constexpr auto ppu_scanline_visible_start = 0;

constexpr auto ppu_scanline_visible_end = 239;

constexpr auto ppu_scanline_postrender = 240;

constexpr auto ppu_scanline_vblank = 241;

constexpr auto ppu_vblank_cycle_delay = 3;

constexpr inline bool is_color_transparent(const uint32_t color) {
  return color == 0;
}

} // namespace

template <typename T>
using memory_mappable_component = jones::memory_mappable_component<T>;

class ppu::impl final {
public:
  impl(memory &cpu_memory, memory &ppu_memory, cpu &cpu, screen::screen *screen)
      : cpu_memory_(cpu_memory), ppu_memory_(ppu_memory), cpu_(cpu), screen_(screen) {
    cpu_memory_.map(std::make_unique<memory_mappable_component<ppu::impl>>(this, "ppu", 0x2000, 0x3FFF));
    cpu_memory_.map(std::make_unique<memory_mappable_component<ppu::impl>>(this, "ppu", 0x4014, 0x4014));
    ppu_memory_.map(std::make_unique<memory_mappable_component<name_table>>(&name_table_, "ppu name_table", name_table_memory_begin, name_table_memory_end));
    ppu_memory_.map(std::make_unique<memory_mappable_component<palette>>(&palette_, "ppu palette", palette_memory_begin, palette_memory_end));
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

  [[nodiscard]] auto peek(const uint16_t address) const -> uint8_t {
    if (address >= 0x2000 && address <= 0x3FFF) {
      return peek_registers(address);
    } else if (address == 0x4014) {
      return peek_object_attribute_memory_dma();
    } else {
      BOOST_STATIC_ASSERT("peek unexpected for ppu");
    }
    return -1;
  }

  auto read(const uint16_t address) -> uint8_t {
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
    check_nmi_trigger();
    process_frame_state();
    update_frame_counters();
    return 1;
  }

  auto get_state() -> ppu_state {
    return ppu_state{
        .cycle = frame_current_cycle_,
        .scanline = frame_current_scanline_,
        .frame = static_cast<size_t>(frame_current_frame_)};
  }

  auto set_state(const ppu_state &state) -> void {
    frame_current_cycle_ = state.cycle;
    frame_current_scanline_ = state.scanline;
    frame_current_frame_ = state.frame;
  }

private:
  auto write_registers(const uint16_t address, const uint8_t data) -> void {
    BOOST_ASSERT_MSG(address >= 0x2000 && address <= 0x3FFF, "write unexpected address for ppu");
    write_register = data;
    auto const address_offset = (address - 0x2000);
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
    io_context_.vram_address_temporary.h_nametable = bit_shift_and<uint8_t>(data, 0, 1);
    io_context_.vram_address_temporary.v_nametable = bit_shift_and<uint8_t>(data, 1, 1);

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
    if (!io_context_.vram_address_latch) {
      io_context_.vram_address_temporary.coarse_x_scroll = bit_shift_and<uint8_t>(data, 3, 5);
      io_context_.fine_x_scroll = bit_shift_and<uint8_t>(data, 0, 3);
    } else {
      io_context_.vram_address_temporary.coarse_y_scroll = bit_shift_and<uint8_t>(data, 3, 5);
      io_context_.vram_address_temporary.fine_y_scroll = bit_shift_and<uint8_t>(data, 0, 3);
    }
    io_context_.vram_address_latch = !io_context_.vram_address_latch;
  }

  auto write_address(const uint8_t data) -> void {
    if (!io_context_.vram_address_latch) {
      auto temp_data = io_context_.vram_address_temporary.value;
      auto data_bits = bit_shift_and<uint8_t>(data, 0, 6);
      bit_shift_set<uint16_t>(temp_data, 8, 7, data_bits);
      io_context_.vram_address_temporary.value = temp_data;
    } else {
      auto temp_data = io_context_.vram_address_temporary.value;
      bit_shift_set<uint16_t>(temp_data, 0, 8, data);
      io_context_.vram_address_temporary.value = temp_data;
      io_context_.vram_address.value = io_context_.vram_address_temporary.value;
    }
    io_context_.vram_address_latch = !io_context_.vram_address_latch;
  }

  auto write_data(const uint8_t data) -> void {
    ppu_memory_.write(io_context_.vram_address.value, data);
    io_context_.vram_address.value += control_register_.is_set(control_flag::VRAM_INCREMENT) ? 32 : 1;
  }

  auto write_object_attribute_memory_dma(const uint8_t data) -> void {
    uint16_t address = static_cast<uint16_t>(data) << 8U;
    for (auto i = 0; i < 256; i++) {
      oam_data_[oam_address_] = cpu_memory_.read(address);
      oam_address_++;
      address++;
    }
    cpu_.idle(ppu_dma_cycles);
    auto const cpu_cycles = cpu_.get_state().cycles.running;
    if (cpu_cycles % 2 == 1) {
      cpu_.idle(1);
    }
  }

  auto read_registers(const uint16_t address) -> uint8_t {
    BOOST_ASSERT_MSG(address >= 0x2000 && address <= 0x3FFF, "read unexpected address for ppu");
    auto const address_offset = (address - 0x2000);
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

  [[nodiscard]] auto read_control() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for control");
    return control_register_.get();
  }

  [[nodiscard]] auto read_mask() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for mask");
    return mask_register_.get();
  }

  auto read_status() -> uint8_t {
    auto const status = (write_register & 0x1FU) | status_register_.get();
    io_context_.vram_address_latch = false;
    status_register_.clear(status_flag::VERTICAL_BLANK_STARTED);
    check_nmi_context();
    return status;
  }

  [[nodiscard]] auto read_object_attribute_memory_address() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for oam address");
    return oam_address_;
  }

  [[nodiscard]] auto read_object_attribute_memory_data() const -> uint8_t {
    return oam_data_[oam_address_];
  }

  [[nodiscard]] auto read_scroll() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for scroll");
    return 0;
  }

  [[nodiscard]] auto read_address() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for address");
    return 0;
  }

  auto read_data() -> uint8_t {
    auto const vram_address = io_context_.vram_address.value;
    auto value = ppu_memory_.read(vram_address);
    if ((vram_address % 0x4000) < palette_background_begin) {
      auto const vram_buffer = io_context_.vram_buffer;
      io_context_.vram_buffer = value;
      value = vram_buffer;
    } else {
      io_context_.vram_buffer = ppu_memory_.read(vram_address - 0x1000);
    }
    io_context_.vram_address.value += control_register_.is_set(control_flag::VRAM_INCREMENT) ? 32 : 1;
    return value;
  }

  [[nodiscard]] auto read_object_attribute_memory_dma() const -> uint8_t {
    BOOST_STATIC_ASSERT("read unexpected for oam dma");
    return 0;
  }

  [[nodiscard]] auto peek_registers(const uint16_t address) const -> uint8_t {
    BOOST_ASSERT_MSG(address >= 0x2000 && address <= 0x3FFF, "peek unexpected address for ppu");
    auto const address_offset = (address - 0x2000);
    switch (address_offset % 8) {
    case 0:
      return read_control();
    case 1:
      return read_mask();
    case 2:
      return peek_status();
    case 3:
      return read_object_attribute_memory_address();
    case 4:
      return read_object_attribute_memory_data();
    case 5:
      return read_scroll();
    case 6:
      return read_address();
    case 7:
      return peek_data();
    }
    return -1;
  }

  [[nodiscard]] auto peek_status() const -> uint8_t {
    if (auto const status = status_register_.get();
        frame_current_scanline_ == ppu_scanline_vblank &&
        frame_current_cycle_ < ppu_vblank_cycle_delay) {
      return status_register{status}.clear(status_flag::VERTICAL_BLANK_STARTED);
    } else {
      return status;
    }
  }

  [[nodiscard]] auto peek_data() const -> uint8_t {
    if (auto const address = io_context_.vram_address.value;
        (address % 0x4000) < palette_background_begin) {
      return io_context_.vram_buffer;
    } else {
      return ppu_memory_.read(address);
    }
  }

  [[nodiscard]] auto peek_object_attribute_memory_dma() const -> uint8_t {
    return read_object_attribute_memory_address();
  }

  auto process_frame_state() -> void {
    auto const state = current_frame_state();
    if (is_frame_state_set(state, ppu_frame_state::STATE_FLAG_VISIBLE)) {
      process_state_flag_visible();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_REG_BG_SHIFT)) {
      process_state_reg_bg_shift();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_REG_SPRITE_SHIFT)) {
      process_state_reg_sprite_shift();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_REG_BG_RELOAD)) {
      process_state_reg_bg_reload();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE)) {
      process_state_vram_fetch_nt_byte();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE)) {
      process_state_vram_fetch_at_byte();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE)) {
      process_state_vram_fetch_bg_low_byte();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE)) {
      process_state_vram_fetch_bg_high_byte();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_FLAG_VBLANK_SET)) {
      process_state_flag_vblank_set();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_FLAG_VBLANK_CLEAR)) {
      process_state_flag_vblank_clear();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_INC_HORI_V)) {
      process_state_loopy_inc_hori_v();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_INC_VERT_V)) {
      process_state_loopy_inc_vert_v();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_SET_HORI_V)) {
      process_state_loopy_set_hori_v();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_LOOPY_SET_VERT_V)) {
      process_state_loopy_set_vert_v();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_SECONDARY_OAM_CLEAR)) {
      process_state_secondary_oam_clear();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_EVALUATE_SPRITE)) {
      process_state_evaluate_sprite();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_VRAM_FETCH_SPRITE_BYTE)) {
      process_state_vram_fetch_sprite_byte();
    }
    if (is_frame_state_set(state, ppu_frame_state::STATE_IDLE)) {
      //
      // Nothing to do.
      //
    }
  }

  auto process_state_vram_fetch_sprite_byte() -> void {
    auto const is_sprites_visible = mask_register_.is_set(mask_flag::SHOW_SPRITES);
    if (!is_sprites_visible) {
      return;
    }
    auto const current_counter = render_context_.sprite_counter;
    auto const current_sprite = reinterpret_cast<ppu_sprite *>(&oam_data_[current_counter]);
    render_context_.sprite_attribute_latches[current_counter] = current_sprite->attributes.value;
    render_context_.sprite_x_position_counters[current_counter] = current_sprite->x;

    render_context_.sprite_counter++;
    if (render_context_.sprite_counter == ppu_sprites_per_line) {
      render_context_.sprite_counter = 0;
    }

    if (current_counter == 0) {
      render_context_.sprite_zero_fetched = render_context_.sprite_zero_evaluated;
    }

    auto const sprite_height = control_register_.is_set(control_flag::SPRITE_SIZE) ? (2 * ppu_tile_height) : ppu_tile_height;
    auto const y = frame_current_scanline_ - current_sprite->y;

    uint16_t sprite_address;
    uint16_t sprite_tile;
    if (!control_register_.is_set(control_flag::SPRITE_SIZE)) {
      sprite_address = (!control_register_.is_set(control_flag::SPRITE_TABLE)) ? pattern_table_zero_memory_begin : pattern_table_one_memory_begin;
      sprite_tile = current_sprite->tile_number;
    } else {
      sprite_address = !(current_sprite->tile_number & 1U) ? pattern_table_zero_memory_begin : pattern_table_one_memory_begin;
      sprite_tile = current_sprite->tile_number & ~1U;
      if ((y >= ppu_tile_height) != current_sprite->attributes.v_flip) {
        sprite_tile++;
      }
    }

    sprite_address += sprite_tile * ppu_tile_size;
    sprite_address += !(current_sprite->attributes.v_flip) ? y % ppu_tile_height : (ppu_tile_height - (y % ppu_tile_height) - 1);

    auto sprite_tile_low = ppu_memory_.read(sprite_address);
    auto sprite_tile_high = ppu_memory_.read(sprite_address + 8);

    if (current_sprite->attributes.h_flip) {
      sprite_tile_low = bit_reverse(sprite_tile_low);
      sprite_tile_high = bit_reverse(sprite_tile_high);
    }

    auto transparent = (current_sprite->y == 0xFF);
    transparent |= (frame_current_scanline_ < current_sprite->y) || (frame_current_scanline_ >= current_sprite->y + sprite_height);
    if (transparent) {
      render_context_.sprite_shift_low[current_counter] = 0;
      render_context_.sprite_shift_high[current_counter] = 0;
      return;
    }

    render_context_.sprite_shift_low[current_counter] = sprite_tile_low;
    render_context_.sprite_shift_high[current_counter] = sprite_tile_high;
  }

  auto process_state_evaluate_sprite() -> void {
    auto const is_sprites_visible = mask_register_.is_set(mask_flag::SHOW_SPRITES);
    if (!is_sprites_visible) {
      return;
    }
    auto const sprite_height = control_register_.is_set(control_flag::SPRITE_SIZE) ? (2 * ppu_tile_height) : ppu_tile_height;
    auto const sprites = reinterpret_cast<ppu_sprite *>(oam_data_.data());

    render_context_.sprite_zero_evaluated = false;

    auto sprite_count = 0;
    for (auto oam_index = 0, sprites_found = 0; sprite_count < ppu_max_sprites; sprite_count++) {
      oam_secondary_data_[oam_index] = sprites[sprite_count].y;

      auto const y = sprites[sprite_count].y;
      if (frame_current_scanline_ < y || frame_current_scanline_ >= y + sprite_height) {
        continue;
      }

      oam_index++;
      oam_secondary_data_[oam_index++] = sprites[sprite_count].values[1];
      oam_secondary_data_[oam_index++] = sprites[sprite_count].values[2];
      oam_secondary_data_[oam_index++] = sprites[sprite_count].values[3];

      if (sprite_count == 0) {
        render_context_.sprite_zero_evaluated = true;
      }

      if (++sprites_found == ppu_sprites_per_line) {
        break;
      }
    }

    auto sprite_overflow = 0;
    while (sprite_count++ < ppu_max_sprites) {
      auto const y = sprites[sprite_count].values[sprite_overflow];
      if ((frame_current_scanline_ >= y) && (frame_current_scanline_ < y + sprite_height)) {
        status_register_.set(status_flag::SPRITE_OVER_FLOW);
        sprite_overflow += 3;
      }
      if (++sprite_overflow >= 4) {
        sprite_overflow -= 4;
      }
    }
  }

  auto process_state_secondary_oam_clear() -> void {
    auto const is_sprites_visible = mask_register_.is_set(mask_flag::SHOW_SPRITES);
    if (!is_sprites_visible) {
      return;
    }
    oam_secondary_data_.fill(0xFF);
  }

  auto process_state_loopy_set_vert_v() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    io_context_.vram_address.coarse_y_scroll = io_context_.vram_address_temporary.coarse_y_scroll;
    io_context_.vram_address.v_nametable = io_context_.vram_address_temporary.v_nametable;
    io_context_.vram_address.fine_y_scroll = io_context_.vram_address_temporary.fine_y_scroll;
  }

  auto process_state_loopy_set_hori_v() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    io_context_.vram_address.coarse_x_scroll = io_context_.vram_address_temporary.coarse_x_scroll;
    io_context_.vram_address.h_nametable = io_context_.vram_address_temporary.h_nametable;
  }

  auto process_state_vram_fetch_bg_high_byte() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    uint16_t address = !control_register_.is_set(control_flag::BACKGROUND_TABLE) ? pattern_table_zero_memory_begin : pattern_table_one_memory_begin;
    address += (render_context_.name_table * ppu_tile_size) + io_context_.vram_address.fine_y_scroll + 8;
    render_context_.background_high = ppu_memory_.read(address);
  }

  auto process_state_vram_fetch_bg_low_byte() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    uint16_t address = !control_register_.is_set(control_flag::BACKGROUND_TABLE) ? pattern_table_zero_memory_begin : pattern_table_one_memory_begin;
    address += (render_context_.name_table * ppu_tile_size) + io_context_.vram_address.fine_y_scroll;
    render_context_.background_low = ppu_memory_.read(address);
  }

  auto process_state_reg_bg_reload() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    bit_shift_set<uint16_t>(render_context_.background_shift_low, 0, 8, render_context_.background_low);
    bit_shift_set<uint16_t>(render_context_.background_shift_high, 0, 8, render_context_.background_high);
    render_context_.attribute_table_latch = render_context_.attribute_table;
  }

  auto process_state_reg_sprite_shift() -> void {
    auto const is_sprites_visible = mask_register_.is_set(mask_flag::SHOW_SPRITES);
    if (!is_sprites_visible) {
      return;
    }
    for (auto i = 0; i < ppu_sprites_per_line; i++) {
      if (render_context_.sprite_x_position_counters[i] == 0) {
        render_context_.sprite_shift_low[i] <<= 1U;
        render_context_.sprite_shift_high[i] <<= 1U;
      }
    }
    for (auto i = 0; i < ppu_sprites_per_line; i++) {
      if (render_context_.sprite_x_position_counters[i] > 0) {
        render_context_.sprite_x_position_counters[i]--;
      }
    }
  }

  auto process_state_reg_bg_shift() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    render_context_.background_shift_low <<= 1U;
    render_context_.background_shift_high <<= 1U;
    render_context_.attribute_table_shift_low <<= 1U;
    render_context_.attribute_table_shift_high <<= 1U;

    auto const attribute_latch = render_context_.attribute_table_latch;
    render_context_.attribute_table_shift_low |= bit_shift_and<uint8_t>(attribute_latch, 0, 1);
    render_context_.attribute_table_shift_high |= bit_shift_and<uint8_t>(attribute_latch, 1, 1);
  }

  auto process_state_loopy_inc_hori_v() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    io_context_.vram_address.coarse_x_scroll++;
    if (io_context_.vram_address.coarse_x_scroll == 0) {
      io_context_.vram_address.h_nametable ^= 1U;
    }
  }

  auto process_state_loopy_inc_vert_v() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    io_context_.vram_address.fine_y_scroll++;
    if (io_context_.vram_address.fine_y_scroll != 0) {
      return;
    }
    io_context_.vram_address.coarse_y_scroll++;
    if (io_context_.vram_address.coarse_y_scroll == (screen_height / ppu_tile_height)) {
      io_context_.vram_address.coarse_y_scroll = 0;
      io_context_.vram_address.v_nametable ^= 1U;
    }
  }

  auto process_state_vram_fetch_nt_byte() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    ppu_vram_address name_table_address = io_context_.vram_address;
    name_table_address.fine_y_scroll = 0;
    const uint16_t address = name_table_memory_begin | name_table_address.value;

    render_context_.name_table = ppu_memory_.read(address);
  }

  auto process_state_vram_fetch_at_byte() -> void {
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (!is_background_visible) {
      return;
    }
    uint16_t const x_scroll = io_context_.vram_address.coarse_x_scroll;
    uint16_t const y_scroll = io_context_.vram_address.coarse_y_scroll;

    ppu_attribute_address attribute_address{};
    attribute_address.high_coarse_x = bit_shift_and<uint16_t>(x_scroll, 2, 3);
    attribute_address.high_coarse_y = bit_shift_and<uint16_t>(y_scroll, 2, 3);
    attribute_address.h_name_table = io_context_.vram_address.h_nametable;
    attribute_address.v_name_table = io_context_.vram_address.v_nametable;

    uint16_t const address = attribute_table_memory_begin | attribute_address.value;
    ppu_attribute const attribute{.value = ppu_memory_.read(address)};

    auto const is_right = io_context_.vram_address.coarse_x_scroll & (1U << 1U);
    auto const is_bottom = io_context_.vram_address.coarse_y_scroll & (1U << 1U);

    render_context_.attribute_table = is_right ? is_bottom ? attribute.bottom_right : attribute.top_right : is_bottom ? attribute.bottom_left : attribute.top_left;
  }

  auto process_state_flag_vblank_set() -> void {
    status_register_.set(status_flag::VERTICAL_BLANK_STARTED);
    check_nmi_context();

    screen_->lock();
    screen_->render();
  }

  auto process_state_flag_vblank_clear() -> void {
    status_register_.clear(status_flag::VERTICAL_BLANK_STARTED);
    status_register_.clear(status_flag::SPRITE_OVER_FLOW);
    status_register_.clear(status_flag::SPRITE_ZERO_HIT);
    check_nmi_context();

    screen_->unlock();
  }

  auto check_nmi_context() -> void {
    static bool previous_nmi{};
    if (!previous_nmi && can_perform_nmi()) {
      nmi_delay_ = 15;
    }
    previous_nmi = can_perform_nmi();
  }

  auto check_nmi_trigger() -> void {
    if (nmi_delay_ > 0) {
      nmi_delay_--;
      if (nmi_delay_ == 0 && can_perform_nmi()) {
        cpu_.interrupt(interrupt_type::NMI);
      }
    }
  }

  auto can_perform_nmi() -> bool {
    return status_register_.is_set(status_flag::VERTICAL_BLANK_STARTED) &&
           control_register_.is_set(control_flag::GENERATE_NMI_ON_VBLANK);
  }

  auto process_state_flag_visible() -> void {
    auto const screen_x_position = frame_current_cycle_ - 2;
    auto const screen_y_position = frame_current_scanline_;

    uint8_t background_palette = 0x00;
    uint16_t background_color = 0x0000;

    auto const is_background_clipped = !(mask_register_.is_set(mask_flag::SHOW_LEFT_BACKGROUND)) && (screen_x_position < ppu_tile_width);
    auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
    if (is_background_visible && !is_background_clipped) {
      auto const background_palette_low = bit_shift_and<uint8_t>(render_context_.attribute_table_shift_low, 7 - io_context_.fine_x_scroll, 1);
      auto const background_palette_high = bit_shift_and<uint8_t>(render_context_.attribute_table_shift_high, 7 - io_context_.fine_x_scroll, 1);
      background_palette = background_palette_low | (background_palette_high << 1);

      auto const background_color_low = bit_shift_and<uint16_t>(render_context_.background_shift_low, 15 - io_context_.fine_x_scroll, 1);
      auto const background_color_high = bit_shift_and<uint16_t>(render_context_.background_shift_high, 15 - io_context_.fine_x_scroll, 1);
      background_color = background_color_low | (background_color_high << 1);
    }

    ppu_sprite_attributes sprite_attributes{};

    uint16_t sprite_color = 0x0000;

    auto const is_sprite_clipped = !(mask_register_.is_set(mask_flag::SHOW_LEFT_SPRITES)) && (screen_x_position < ppu_tile_width);
    auto const is_sprite_visible = mask_register_.is_set(mask_flag::SHOW_SPRITES);
    if (is_sprite_visible && !is_sprite_clipped) {
      for (auto i = 0; i < ppu_sprites_per_line; i++) {
        if (render_context_.sprite_x_position_counters[i] > 0) {
          continue;
        }
        auto const sprite_color_low = bit_shift_and<uint8_t>(render_context_.sprite_shift_low[i], 7, 1);
        auto const sprite_color_high = bit_shift_and<uint8_t>(render_context_.sprite_shift_high[i], 7, 1);
        auto const possible_sprite_color = sprite_color_low | (sprite_color_high << 1);
        if (is_color_transparent(possible_sprite_color)) {
          continue;
        }
        auto sprite_zero_hit = (i == 0);
        sprite_zero_hit &= (render_context_.sprite_zero_fetched);
        sprite_zero_hit &= (screen_x_position != 255);
        sprite_zero_hit &= (!is_color_transparent(background_color));
        sprite_zero_hit &= (status_register_.is_set(status_flag::SPRITE_ZERO_HIT));
        if (sprite_zero_hit) {
          status_register_.set(status_flag::SPRITE_ZERO_HIT);
        }
        sprite_color = possible_sprite_color;
        sprite_attributes.value = render_context_.sprite_attribute_latches[i];
        break;
      }
    }

    auto background_has_priority = true;
    if (is_color_transparent(background_color) && !is_color_transparent(sprite_color)) {
      background_has_priority = false;
    }
    if (!is_color_transparent(background_color) && !is_color_transparent(sprite_color) && !sprite_attributes.priority) {
      background_has_priority = false;
    }

    auto const pixel_color = background_has_priority ? background_color : sprite_color;
    auto const pixel_palette = background_has_priority ? background_palette : sprite_attributes.palette;

    auto palette_address = background_has_priority ? palette_background_begin : palette_sprite_begin;
    if (!is_color_transparent(pixel_color)) {
      palette_address += (palette_entries * pixel_palette) + pixel_color;
    }
    auto const palette = ppu_memory_.read(palette_address);
    auto const pixel = get_palette_pixel(palette);
    screen_->set_pixel(screen_x_position, screen_y_position, pixel);
  }

  auto update_frame_counters() -> void {
    update_cycle_counter();
    update_scanline_counter();
    update_frame_counter();
  }

  auto update_cycle_counter() -> void {
    frame_current_cycle_ += 1;
    if (frame_current_cycle_ > ppu_max_cycles) {
      frame_current_cycle_ = 0;
    }
  }

  auto update_frame_counter() -> void {
    if (frame_current_cycle_ == ppu_max_cycles &&
        frame_current_scanline_ == ppu_scanline_visible_end) {
      frame_current_frame_ += 1;
      auto const is_odd_frame = frame_current_frame_ % 2 == 1;
      auto const is_background_visible = mask_register_.is_set(mask_flag::SHOW_BACKGROUND);
      if (is_odd_frame && is_background_visible) {
        frame_current_cycle_ += 1;
      }
    }
  }

  auto update_scanline_counter() -> void {
    if (frame_current_cycle_ == 0) {
      frame_current_scanline_ += 1;
      if (frame_current_scanline_ >= ppu_max_scanlines) {
        frame_current_scanline_ = 0;
      }
    }
  }

  auto initialize_frame_scanlines() -> void {
    auto const scanline_cycles = std::vector<ppu_frame_state_mask>(ppu_num_cycles, 0);
    frame_scanlines_ = std::vector<ppu_frame_cycles>(ppu_max_scanlines, scanline_cycles);
  }

  auto initialize_prerender_scanline() -> void {

    auto &scanline = frame_scanlines_[ppu_scanline_prerender];

    scanline[0] |= static_cast<uint32_t>(ppu_frame_state::STATE_IDLE);

    scanline[1] |= static_cast<uint32_t>(ppu_frame_state::STATE_FLAG_VBLANK_CLEAR);

    for (auto i = 1; i <= 256; i += 8) {
      scanline[i + 0] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
      scanline[i + 2] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE);
      scanline[i + 4] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE);
      scanline[i + 6] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE);
    }

    for (auto i = 2; i <= 257; i++) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_SHIFT);
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_SPRITE_SHIFT);
    }

    for (auto i = 9; i <= 257; i += 8) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_RELOAD);
    }

    for (auto i = 8; i <= 256; i += 8) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_HORI_V);
    }

    scanline[256] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_VERT_V);
    scanline[257] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_SET_HORI_V);

    for (auto i = 257; i < 320; i += 8) {
      scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_SPRITE_BYTE);
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

      for (auto i = 1; i <= 256; i += 8) {
        scanline[i + 0] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_NT_BYTE);
        scanline[i + 2] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_AT_BYTE);
        scanline[i + 4] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_LOW_BYTE);
        scanline[i + 6] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_BG_HIGH_BYTE);
      }

      for (auto i = 2; i <= 257; i++) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_SHIFT);
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_SPRITE_SHIFT);
      }

      for (auto i = 9; i <= 257; i += 8) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_REG_BG_RELOAD);
      }

      for (auto i = 8; i <= 256; i += 8) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_HORI_V);
      }

      scanline[256] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_INC_VERT_V);
      scanline[257] |= static_cast<uint32_t>(ppu_frame_state::STATE_LOOPY_SET_HORI_V);

      for (auto i = 257; i < 320; i += 8) {
        scanline[i] |= static_cast<uint32_t>(ppu_frame_state::STATE_VRAM_FETCH_SPRITE_BYTE);
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

  cpu &cpu_;

  screen::screen *const screen_;

  control_register control_register_{};

  mask_register mask_register_{};

  status_register status_register_{};

  uint8_t write_register{};

  uint8_t oam_address_{};

  std::array<uint8_t, ppu_oam_size> oam_data_ = {0};

  std::array<uint8_t, ppu_oam_secondary_size> oam_secondary_data_ = {0};

  palette palette_{};

  name_table name_table_{};

  uint16_t frame_current_cycle_{};

  uint16_t frame_current_scanline_{};

  uint64_t frame_current_frame_{1};

  ppu_frame_scanlines frame_scanlines_{};

  ppu_render_context render_context_{};

  ppu_io_context io_context_{};

  uint8_t nmi_delay_{};
};

ppu::ppu(jones::memory &cpu_memory, jones::memory &ppu_memory, cpu &cpu, screen::screen *screen)
    : impl_(new impl(cpu_memory, ppu_memory, cpu, screen)) {
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

auto ppu::peek(const uint16_t address) const -> uint8_t {
  return impl_->peek(address);
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

auto ppu::set_state(const ppu_state &state) -> void {
  return impl_->set_state(state);
}
