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
#include <boost/core/ignore_unused.hpp>

#include "apu.hh"
#include "memory.hh"

using namespace jones;

class apu::impl final {
public:
  explicit impl(memory &memory) : cpu_memory_(memory) {
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu::impl>>(this, "apu", 0x4000, 0x4013));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu::impl>>(this, "apu", 0x4015, 0x4015));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu::impl>>(this, "apu", 0x4018, 0x401F));
  }

  auto step() -> uint8_t {
    return 0;
  }

  auto peek(uint16_t const address) const -> uint8_t {
    return read(address);
  }

  auto read(uint16_t const address) const -> uint8_t {
    boost::ignore_unused(address);
    return 0;
  }

  auto write(uint16_t const address, uint8_t const data) -> void {
    boost::ignore_unused(address, data);
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
