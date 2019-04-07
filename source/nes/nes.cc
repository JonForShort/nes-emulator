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
#include <memory.hh>

#include "cpu.hh"
#include "nes.hh"
#include "nes_rom.hh"

using namespace jones;

class cpu_memory_map : public memory_mappable {
public:
  cpu_memory_map(cpu &cpu) : cpu_(cpu) {}

  uint16_t start_address() override {
    return 0x0000;
  }

  uint16_t end_address() override {
    return 0x1FFF;
  }

  uint8_t read(const uint16_t address) override {
    return cpu_.read(address);
  }

  void write(const uint16_t address, const uint8_t data) override {
    cpu_.write(address, data);
  }

private:
  cpu &cpu_;
};

class nes::nes_impl {
public:
  nes_impl() : memory_(), cpu_(memory_) {
    memory_.map(std::make_unique<cpu_memory_map>(cpu_));
  }

  void load(const nes_rom &rom) {
    boost::ignore_unused(rom);
  }

  void run() {
  }

  void reset() {
  }

  void trace(const char *trace_file) {
    boost::ignore_unused(trace_file);
  }

private:
  memory memory_;
  cpu cpu_;
};

nes::nes() noexcept
    : pimpl_(std::make_unique<nes_impl>()) {
}

void nes::load(const nes_rom &rom) {
  pimpl_->load(rom);
}

void nes::run() {
  pimpl_->run();
}

void nes::reset() {
  pimpl_->reset();
}

void nes::trace(const char *trace_file) {
  pimpl_->trace(trace_file);
}
