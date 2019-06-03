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
#include "ppu_frame.hh"
#include "status_register.hh"

using namespace jones::ppu;

template <typename T>
using memory_mappable_component = jones::memory_mappable_component<T>;

class ppu::impl final {
public:
  impl(memory &cpu_memory, memory &ppu_memory)
      : cpu_memory_(cpu_memory), ppu_memory_(ppu_memory), ppu_frame_(mask_register_) {
    ppu_memory_.map(std::make_unique<memory_mappable_component<pattern_table>>(&pattern_table_, pattern_table_memory_begin, pattern_table_memory_end));
    ppu_memory_.map(std::make_unique<memory_mappable_component<name_table>>(&name_table_, name_table_memory_begin, name_table_memory_end));
    ppu_memory_.map(std::make_unique<memory_mappable_component<palette>>(&palette_, palette_memory_begin, palette_memory_end));
  }

  uint8_t step() {
    return ppu_frame_.step();
  }

  uint8_t read(const uint16_t address) const {
    if (address >= 0x2000 && address <= 0x3FFF) {
      return read_registers(address);
    } else if (address == 0x4014) {
      return read_object_attribute_memory_dma();
    } else {
      BOOST_STATIC_ASSERT("read unexpected for ppu");
    }
    return -1;
  }

  uint8_t read_registers(const uint16_t address) const {
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

  uint8_t read_control() const {
    BOOST_STATIC_ASSERT("read unexpected for control");
    return control_register_.get();
  }

  uint8_t read_mask() const {
    BOOST_STATIC_ASSERT("read unexpected for mask");
    return mask_register_.get();
  }

  uint8_t read_status() const {
    return status_register_.get();
  }

  uint8_t read_object_attribute_memory_address() const {
    BOOST_STATIC_ASSERT("read unexpected for oam address");
    return oam_address_;
  }

  uint8_t read_object_attribute_memory_data() const {
    return oam_data_[oam_address_];
  }

  uint8_t read_scroll() const {
    return 0;
  }

  uint8_t read_address() const {
    return 0;
  }

  uint8_t read_data() const {
    return 0;
  }

  uint8_t read_object_attribute_memory_dma() const {
    return 0;
  }

  void write(const uint16_t address, const uint8_t data) {
    if (address >= 0x2000 && address <= 0x3FFF) {
      write_registers(address, data);
    } else if (address == 0x4014) {
      write_object_attribute_memory_dma(data);
    } else {
      BOOST_STATIC_ASSERT("write unexpected for ppu");
    }
  }

  void write_registers(const uint16_t address, const uint8_t data) {
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

  void write_control(const uint8_t data) {
    control_register_.set(data);
  }

  void write_mask(const uint8_t data) {
    mask_register_.set(data);
  }

  void write_status(const uint8_t data) {
    BOOST_STATIC_ASSERT("write unexpected for status");
    status_register_.set(data);
  }

  void write_object_attribute_memory_address(const uint8_t data) {
    oam_address_ = data;
  }

  void write_object_attribute_memory_data(const uint8_t data) {
    oam_data_[oam_address_] = data;
    oam_address_ += 1;
  }

  void write_scroll(const uint8_t data) {
    boost::ignore_unused(data);
  }

  void write_address(const uint8_t data) {
    boost::ignore_unused(data);
  }

  void write_data(const uint8_t data) {
    boost::ignore_unused(data);
  }

  void write_object_attribute_memory_dma(const uint8_t data) {
    boost::ignore_unused(data);
  }

  void initialize() {
    ppu_frame_.initialize();
  }

  void uninitialize() {
  }

  ppu_state get_state() {
    return ppu_state{
        .cycle = ppu_frame_.cycle(),
        .scanline = ppu_frame_.scanline(),
        .frame = ppu_frame_.frame()};
  }

  auto get_buffer() const -> auto {
    return ppu_frame_.buffer();
  }

  auto is_buffer_ready() const -> auto {
    return ppu_frame_.buffer_ready();
  }

private:
  memory &cpu_memory_;

  memory &ppu_memory_;

  control_register control_register_{};

  mask_register mask_register_{};

  status_register status_register_{};

  uint8_t oam_address_{};

  uint8_t oam_data_[0xFF] = {0};

  ppu_frame ppu_frame_;

  palette palette_{};

  name_table name_table_{};

  pattern_table pattern_table_{};
};

ppu::ppu(jones::memory &cpu_memory, jones::memory &ppu_memory) : impl_(new impl(cpu_memory, ppu_memory)) {}

ppu::~ppu() = default;

auto ppu::step() -> uint8_t {
  return impl_->step();
}

auto ppu::read(const uint16_t address) const -> uint8_t {
  return impl_->read(address);
}

auto ppu::write(const uint16_t address, const uint8_t data) -> void {
  impl_->write(address, data);
}

auto ppu::initialize() -> void {
  impl_->initialize();
}

auto ppu::uninitialize() -> void {
  impl_->uninitialize();
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