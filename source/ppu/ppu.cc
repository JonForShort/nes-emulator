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

#include "memory.hh"
#include "ppu.hh"

using namespace jones;

class ppu::impl final {
public:
  explicit impl(const memory &memory) : memory_(memory) {}

  uint8_t step() {
    return 0;
  }

  uint8_t read(uint16_t address) {
    boost::ignore_unused(address);
    return 0;
  }

  void write(uint16_t address, uint8_t data) {
    boost::ignore_unused(address);
    boost::ignore_unused(data);
  }

  void initialize() {
    boost::ignore_unused(memory_);
  }

  void uninitialize() {
  }

private:
  const memory &memory_;
};

ppu::ppu(const jones::memory &memory)
    : impl_(new impl(memory)) {
}

ppu::~ppu() = default;

uint8_t ppu::step() {
  return impl_->step();
}

uint8_t ppu::read(const uint16_t address) const {
  return impl_->read(address);
}

void ppu::write(const uint16_t address, const uint8_t data) {
  impl_->write(address, data);
}

void ppu::initialize() {
}

void ppu::uninitialize() {
}

ppu_state ppu::get_state() const {
  return ppu_state{.cycles = 0};
}