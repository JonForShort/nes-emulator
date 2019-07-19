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
#include <boost/static_assert.hpp>

#include "apu.hh"
#include "memory.hh"

//
// APU implementation is largely influenced by the following documents.
//
// https://wiki.nesdev.com/w/index.php/APU
//

using namespace jones;

class apu::impl final {
public:
  explicit impl(memory &memory) : cpu_memory_(memory) {
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu::impl>>(this, "apu", 0x4000, 0x4013));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu::impl>>(this, "apu", 0x4015, 0x4015));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu::impl>>(this, "apu", 0x4017, 0x4017));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu::impl>>(this, "apu", 0x4018, 0x401F));
  }

  auto initialize() -> void {
    //
    // nothing to do.
    //
  }

  auto uninitialize() -> void {
    //
    // nothing to do.
    //
  }

  auto step() -> uint8_t {
    return 0;
  }

  auto peek(uint16_t const address) const -> uint8_t {
    return read(address);
  }

  auto read(uint16_t const address) const -> uint8_t {
    auto const address_offset = (address - 0x4000);
    switch (address_offset) {
    case 0x15:
      return read_status();
    default:
      BOOST_STATIC_ASSERT("unexpected read address");
      return 0;
    }
  }

  auto read_status() const -> uint8_t {
    return 0;
  }

  auto write(uint16_t const address, uint8_t const data) -> void {
    auto const address_offset = (address - 0x4000);
    switch (address_offset) {
    case 0x00:
      return write_pulse_one_control(data);
    case 0x01:
      return write_pulse_one_sweep(data);
    case 0x02:
      return write_pulse_one_timer_low(data);
    case 0x03:
      return write_pulse_one_timer_high(data);
    case 0x04:
      return write_pulse_two_control(data);
    case 0x05:
      return write_pulse_two_sweep(data);
    case 0x06:
      return write_pulse_two_timer_low(data);
    case 0x07:
      return write_pulse_two_timer_high(data);
    case 0x08:
      return write_triangle_control(data);
    case 0x09:
      // Unused
      break;
    case 0x0A:
      return write_triangle_timer_low(data);
    case 0x0B:
      return write_triangle_timer_high(data);
    case 0x0C:
      return write_noise_control(data);
    case 0x0D:
      // Unused
      break;
    case 0x0E:
      return write_noise_period(data);
    case 0x0F:
      return write_noise_length(data);
    case 0x10:
      return write_dmc_control(data);
    case 0x11:
      return write_dmc_value(data);
    case 0x12:
      return write_dmc_address(data);
    case 0x13:
      return write_dmc_length(data);
    case 0x15:
      return write_control(data);
    case 0x17:
      return write_frame_counter(data);
    default:
      BOOST_STATIC_ASSERT("unexpected write address");
      return;
    }
  }

  auto write_pulse_one_control(uint8_t const data) -> void {
    (void)data;
  }

  auto write_pulse_one_sweep(uint8_t const data) -> void {
    (void)data;
  }

  auto write_pulse_one_timer_low(uint8_t const data) -> void {
    (void)data;
  }

  auto write_pulse_one_timer_high(uint8_t const data) -> void {
    (void)data;
  }

  auto write_pulse_two_control(uint8_t const data) -> void {
    (void)data;
  }

  auto write_pulse_two_sweep(uint8_t const data) -> void {
    (void)data;
  }

  auto write_pulse_two_timer_low(uint8_t const data) -> void {
    (void)data;
  }

  auto write_pulse_two_timer_high(uint8_t const data) -> void {
    (void)data;
  }

  auto write_triangle_control(uint8_t const data) -> void {
    (void)data;
  }

  auto write_triangle_timer_low(uint8_t const data) -> void {
    (void)data;
  }

  auto write_triangle_timer_high(uint8_t const data) -> void {
    (void)data;
  }

  auto write_noise_control(uint8_t const data) -> void {
    (void)data;
  }

  auto write_noise_period(uint8_t const data) -> void {
    (void)data;
  }

  auto write_noise_length(uint8_t const data) -> void {
    (void)data;
  }

  auto write_dmc_control(uint8_t const data) -> void {
    (void)data;
  }

  auto write_dmc_value(uint8_t const data) -> void {
    (void)data;
  }

  auto write_dmc_address(uint8_t const data) -> void {
    (void)data;
  }

  auto write_dmc_length(uint8_t const data) -> void {
    (void)data;
  }

  auto write_control(uint8_t const data) -> void {
    (void)data;
  }

  auto write_frame_counter(uint8_t const data) -> void {
    (void)data;
  }

private:
  memory &cpu_memory_;
};

apu::apu(memory &memory)
    : impl_(new apu::impl(memory)) {
}

apu::~apu() = default;

auto apu::step() -> uint8_t {
  return impl_->step();
}

auto apu::peek(const uint16_t address) const -> uint8_t {
  return impl_->peek(address);
}

auto apu::read(const uint16_t address) const -> uint8_t {
  return impl_->read(address);
}

auto apu::write(uint16_t const address, uint8_t const data) -> void {
  impl_->write(address, data);
}

auto apu::initialize() -> void {
  impl_->initialize();
}

auto apu::uninitialize() -> void {
  impl_->uninitialize();
}
