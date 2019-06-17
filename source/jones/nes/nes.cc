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
#include <atomic>
#include <boost/core/ignore_unused.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>

#include "apu.hh"
#include "cartridge.hh"
#include "cpu.hh"
#include "debugger.hh"
#include "memory.hh"
#include "nes.hh"
#include "nes_components.hh"
#include "ppu.hh"
#include "screen.hh"
#include "screen_proxy.hh"

using namespace jones;
using namespace boost::filesystem;

class nes::impl {
public:
  impl() : screen_(std::make_unique<screen_proxy>()),
           apu_(cpu_memory_),
           cpu_(cpu_memory_),
           ppu_(cpu_memory_, ppu_memory_, cpu_, screen_.get()),
           cartridge_(cpu_memory_, ppu_memory_),
           controller_one_(std::make_unique<controller::controller>(cpu_memory_)),
           controller_two_(std::make_unique<controller::controller>(cpu_memory_)) {
    cpu_memory_.map(std::make_unique<memory_mappable_component<memory_ram>>(&ram_, 0x0000, 0x1FFF));
    cpu_memory_.map(std::make_unique<memory_mappable_component<ppu::ppu>>(&ppu_, 0x2000, 0x3FFF));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu>>(&apu_, 0x4000, 0x4015));
    cpu_memory_.map(std::make_unique<memory_mappable_component<cpu>>(&cpu_, 0x4000, 0x4015));
    cpu_memory_.map(std::make_unique<memory_mappable_component<controller::controller>>(controller_one_.get(), 0x4016, 0x4016));
    cpu_memory_.map(std::make_unique<memory_mappable_component<controller::controller>>(controller_two_.get(), 0x4017, 0x4017));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu>>(&apu_, 0x4018, 0x401F));
    cpu_memory_.map(std::make_unique<memory_mappable_component<cpu>>(&cpu_, 0x4018, 0x401F));
  }

  auto load(const char *rom_path) -> bool {
    if (exists(rom_path)) {
      return cartridge_.attach(rom_path);
    }
    return false;
  }

  auto run(const size_t step_limit) -> void {
    is_running_ = true;
    initialize_components();
    notify_listener(nes_listener::event::ON_RUN_STARTED);
    for (size_t step_count = 0; is_running_ && (step_count < step_limit || step_limit == 0); step_count++) {
      step();
      notify_listener(nes_listener::event::ON_RUN_STEP);
    }
    notify_listener(nes_listener::event::ON_RUN_FINISHED);
    uninitialize_components();
  }

  auto stop() -> void {
    is_running_ = false;
  }

  auto reset() -> void {
    initialize_components();
  }

  auto controller_one() const -> controller::controller_ptr {
    return controller_one_.get();
  }

  auto controller_two() const -> controller::controller_ptr {
    return controller_two_.get();
  }

  auto attach_screen(std::unique_ptr<screen::screen> screen) -> void {
    screen_->attach_screen(std::move(screen));
  }

  auto read(const uint16_t address) -> uint8_t {
    return cpu_.read(address);
  }

  auto write(const uint16_t address, const uint8_t data) -> void {
    cpu_.write(address, data);
  }

  auto set_listener(nes_listener *listener) -> void {
    listener_ = listener;
  }

private:
  auto initialize_components() -> void {
    ppu_.initialize();
    cpu_.initialize();
    apu_.initialize();
  }

  auto uninitialize_components() -> void {
    ppu_.uninitialize();
    cpu_.uninitialize();
    apu_.uninitialize();
  }

  auto notify_listener(const nes_listener::event event) -> void {
    if (listener_ != nullptr) {
      listener_->on_event(event, get_state());
    }
  }

  auto get_state() -> nes_state {
    const auto ppu_state = ppu_.get_state();
    const auto cpu_state = cpu_.get_state();

    nes_state state{};

    state.ppu_scanline = ppu_state.scanline;
    state.ppu_cycle = ppu_state.cycle;
    state.ppu_frame = ppu_state.frame;

    state.registers.A = cpu_state.registers.A;
    state.registers.X = cpu_state.registers.X;
    state.registers.Y = cpu_state.registers.Y;
    state.registers.SR = cpu_state.registers.SR;
    state.registers.SP = cpu_state.registers.SP;
    state.registers.PC = cpu_state.registers.PC;

    state.cpu_cycle = cpu_state.cycles;
    state.instruction = cpu_state.instruction;
    state.instruction_bytes = cpu_state.instruction_bytes;

    return state;
  }

  auto step() -> void {
    const auto cpu_cycles = cpu_.step();
    const auto ppu_cycles = cpu_cycles * 3;
    const auto apu_cycles = cpu_cycles;
    for (auto i = 0; i < ppu_cycles; i++) {
      ppu_.step();
    }
    for (auto i = 0; i < apu_cycles; i++) {
      apu_.step();
    }
  }

private:
  std::atomic<bool> is_running_{};

  std::unique_ptr<screen_proxy> screen_{};

  memory cpu_memory_{};

  memory ppu_memory_{};

  apu apu_;

  cpu cpu_;

  ppu::ppu ppu_;

  cartridge cartridge_;

  memory_ram ram_{};

  std::unique_ptr<controller::controller> controller_one_{};

  std::unique_ptr<controller::controller> controller_two_{};

  nes_listener *listener_{};
};

nes::nes() noexcept
    : pimpl_(std::make_unique<impl>()) {
}

nes::~nes() = default;

bool nes::load(const char *rom_path) {
  return pimpl_->load(rom_path);
}

void nes::run(const size_t cycle_count) {
  pimpl_->run(cycle_count);
}

void nes::stop() {
  pimpl_->stop();
}

void nes::reset() {
  pimpl_->reset();
}

controller::controller_ptr nes::controller_one() {
  return pimpl_->controller_one();
}

controller::controller_ptr nes::controller_two() {
  return pimpl_->controller_two();
}

void nes::attach_screen(std::unique_ptr<screen::screen> screen) {
  pimpl_->attach_screen(std::move(screen));
}

class debugger::impl final {
public:
  explicit impl(const nes &nes) : nes_(nes) {}

  auto read(const uint16_t address) -> uint8_t {
    return nes_.pimpl_->read(address);
  }

  auto write(uint16_t address, uint8_t data) -> void {
    nes_.pimpl_->write(address, data);
  }

  auto set_listener(jones::nes_listener *listener) -> void {
    nes_.pimpl_->set_listener(listener);
  }

private:
  const nes &nes_;
};

debugger::debugger(const nes &nes) : impl_(new debugger::impl(nes)) {
}

debugger::~debugger() = default;

auto debugger::read(const uint16_t address) -> uint8_t {
  return impl_->read(address);
}

auto debugger::write(uint16_t address, uint8_t data) -> void {
  impl_->write(address, data);
}

auto debugger::set_listener(jones::nes_listener *listener) -> void {
  impl_->set_listener(listener);
}
