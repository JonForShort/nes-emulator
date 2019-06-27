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
#include "log.hh"
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
  explicit impl(const char *const rom_path) noexcept
      : rom_path_(rom_path),
        screen_(std::make_unique<screen_proxy>()),
        apu_(cpu_memory_),
        cpu_(cpu_memory_),
        ppu_(cpu_memory_, ppu_memory_, cpu_, screen_.get()),
        cartridge_(cpu_memory_, ppu_memory_),
        controller_one_(std::make_unique<controller::controller>(cpu_memory_)),
        controller_two_(std::make_unique<controller::controller>(cpu_memory_)) {
    cpu_memory_.map(std::make_unique<memory_mappable_component<memory_ram>>(&ram_, "memory_ram", 0x0000, 0x1FFF));
    cpu_memory_.map(std::make_unique<memory_mappable_component<ppu::ppu>>(&ppu_, "ppu", 0x2000, 0x3FFF));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu>>(&apu_, "apu", 0x4000, 0x4015));
    cpu_memory_.map(std::make_unique<memory_mappable_component<cpu>>(&cpu_, "cpu", 0x4000, 0x4015));
    cpu_memory_.map(std::make_unique<memory_mappable_component<controller::controller>>(controller_one_.get(), "controller_one", 0x4016, 0x4016));
    cpu_memory_.map(std::make_unique<memory_mappable_component<controller::controller>>(controller_two_.get(), "controller_two", 0x4017, 0x4017));
    cpu_memory_.map(std::make_unique<memory_mappable_component<apu>>(&apu_, "apu", 0x4018, 0x401F));
    cpu_memory_.map(std::make_unique<memory_mappable_component<cpu>>(&cpu_, "cpu", 0x4018, 0x401F));
  }

  auto power() -> bool {
    if (is_power_on_) {
      return turn_power_off();
    } else {
      return turn_power_on();
    }
  }

  auto reset() -> bool {
    return true;
  }

  auto run(const size_t step_limit) -> void {
    is_running_ = true;
    notify_listener(nes_listener::event::ON_RUN_STARTED);
    for (size_t step_count = 0; is_running_ && (step_count < step_limit || step_limit == 0); step_count++) {
      step();
      notify_listener(nes_listener::event::ON_RUN_STEP);
    }
    notify_listener(nes_listener::event::ON_RUN_FINISHED);
    is_running_ = false;
  }

  auto stop() -> void {
    is_running_ = false;
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
  auto initialize_components() -> bool {
    ppu_.initialize();
    cpu_.initialize();
    apu_.initialize();
    return true;
  }

  auto uninitialize_components() -> bool {
    ppu_.uninitialize();
    cpu_.uninitialize();
    apu_.uninitialize();
    return true;
  }

  auto turn_power_on() -> bool {
    const auto is_rom_loaded = cartridge_.attach(rom_path_.c_str());
    const auto is_components_initialized = initialize_components();
    is_power_on_ = is_components_initialized && is_rom_loaded;
    if (is_power_on_) {
      log_memory_mappings();
      notify_listener(nes_listener::event::ON_POWER_ON);
      step_power_on();
    }
    return is_power_on_;
  }

  auto step_power_on() -> void {
    //
    // PPU performs 30 steps on power up.
    //
    for (auto i = 0; i < 30; i++) {
      ppu_.step();
    }
    //
    // APU performs 10 steps on power up.
    //
    for (auto i = 0; i < 10; i++) {
      apu_.step();
    }
  }

  auto turn_power_off() -> bool {
    const auto is_components_uninitialized = uninitialize_components();
    const auto is_cartridge_detached = cartridge_.attach(nullptr);
    const auto is_power_off = is_components_uninitialized && is_cartridge_detached;
    if (is_power_off) {
      is_power_on_ = false;
      notify_listener(nes_listener::event::ON_POWER_OFF);
    }
    return true;
  }

  auto notify_listener(const nes_listener::event &event) -> void {
    if (listener_ != nullptr) {
      auto state = get_state();
      listener_->on_event(event, state);
      set_state(state);
    }
  }

  auto log_memory_mappings() const -> void {
    std::ostringstream cpu_memory_mappings;
    cpu_memory_.print(cpu_memory_mappings);
    LOG_DEBUG << cpu_memory_mappings.str();

    std::ostringstream ppu_memory_mappings;
    ppu_memory_.print(ppu_memory_mappings);
    LOG_DEBUG << cpu_memory_mappings.str();
  }

  auto get_state() -> nes_state {
    const auto ppu_state = ppu_.get_state();
    const auto cpu_state = cpu_.get_state();

    nes_state state{};

    state.ppu.scanline = ppu_state.scanline;
    state.ppu.cycle = ppu_state.cycle;
    state.ppu.frame = ppu_state.frame;

    state.cpu.registers.A = cpu_state.registers.A;
    state.cpu.registers.X = cpu_state.registers.X;
    state.cpu.registers.Y = cpu_state.registers.Y;
    state.cpu.registers.SR = cpu_state.registers.SR;
    state.cpu.registers.SP = cpu_state.registers.SP;
    state.cpu.registers.PC = cpu_state.registers.PC;
    state.cpu.cycle = cpu_state.cycles;

    state.instruction.instruction = cpu_state.instruction.instruction;
    state.instruction.instruction_bytes = cpu_state.instruction.instruction_bytes;

    state.instruction.memory.address = cpu_state.instruction.memory.address;
    state.instruction.memory.value = cpu_state.instruction.memory.value;
    state.instruction.memory.is_indirect = cpu_state.instruction.memory.is_indirect;

    return state;
  }

  auto set_state(const nes_state &state) -> void {
    auto cpu_state = cpu_.get_state();
    cpu_state.registers.PC = state.cpu.registers.PC;
    cpu_state.registers.SP = state.cpu.registers.SP;
    cpu_state.registers.SR = state.cpu.registers.SR;
    cpu_state.registers.A = state.cpu.registers.A;
    cpu_state.registers.X = state.cpu.registers.X;
    cpu_state.registers.Y = state.cpu.registers.Y;
    cpu_.set_state(cpu_state);

    auto ppu_state = ppu_.get_state();
    ppu_state.cycle = state.ppu.cycle;
    ppu_state.scanline = state.ppu.scanline;
    ppu_state.frame = state.ppu.frame;
    ppu_.set_state(ppu_state);
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

  std::atomic<bool> is_power_on_{};

  const std::string rom_path_{};

  std::unique_ptr<screen_proxy> screen_{};

  memory cpu_memory_{"cpu"};

  memory ppu_memory_{"ppu"};

  apu apu_;

  cpu cpu_;

  ppu::ppu ppu_;

  cartridge cartridge_;

  memory_ram ram_{};

  std::unique_ptr<controller::controller> controller_one_{};

  std::unique_ptr<controller::controller> controller_two_{};

  nes_listener *listener_{};
};

nes::nes(const char *rom_path) noexcept
    : pimpl_(std::make_unique<impl>(rom_path)) {
}

nes::~nes() = default;

auto nes::power() -> bool {
  return pimpl_->power();
}

auto nes::reset() -> bool {
  return pimpl_->reset();
}

auto nes::run(const size_t cycle_count) -> void {
  return pimpl_->run(cycle_count);
}

auto nes::stop() -> void {
  return pimpl_->stop();
}

auto nes::controller_one() -> controller::controller_ptr {
  return pimpl_->controller_one();
}

auto nes::controller_two() -> controller::controller_ptr {
  return pimpl_->controller_two();
}

auto nes::attach_screen(std::unique_ptr<screen::screen> screen) -> void {
  return pimpl_->attach_screen(std::move(screen));
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
  //
  // nothing to do.
  //
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
