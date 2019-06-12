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
    cpu_memory_.map(std::make_unique<memory_mappable_component<memory_sram>>(&sram_, 0x6000, 0x7FFF));
    cpu_memory_.map(std::make_unique<memory_mappable_component<cartridge>>(&cartridge_, 0x8000, 0xFFFF));
  }

  auto load(const char *rom_path) -> bool {
    if (boost::filesystem::exists(rom_path)) {
      return cartridge_.attach(rom_path);
    }
    return false;
  }

  auto run(const size_t step_limit) -> void {
    initialize_components();
    is_running_ = true;
    trace_step();
    for (size_t step_count = 0; is_running_ && (step_count < step_limit || step_limit == 0); step_count++) {
      const auto cpu_cycles = cpu_.step();
      const auto ppu_cycles = cpu_cycles * 3;
      const auto apu_cycles = cpu_cycles;
      for (auto i = 0; i < ppu_cycles; i++) {
        ppu_.step();
      }
      for (auto i = 0; i < apu_cycles; i++) {
        apu_.step();
      }
      trace_step();
    }
    trace_done();
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

  auto trace(const char *trace_file) -> void {
    trace_file_ = std::ofstream{trace_file};
  }

  auto read(const uint16_t address) -> uint8_t {
    return cpu_.read(address);
  }

  auto write(const uint16_t address, const uint8_t data) -> void {
    cpu_.write(address, data);
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

  auto trace_step() -> void {
    if (trace_file_.is_open()) {
      const auto cpu_state = cpu_.get_state();

      trace_file_ << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << cpu_state.registers.PC << "  ";

      for (const auto &i : cpu_state.instruction_bytes) {
        trace_file_ << std::right << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<uint16_t>(i) << " ";
      }

      const size_t instruction_bytes_padding = 10 - (cpu_state.instruction_bytes.size() * 3);
      for (size_t i = 0; i < instruction_bytes_padding; i++) {
        trace_file_ << " ";
      }

      trace_file_ << cpu_state.instruction;

      const size_t instruction_padding = 32 - cpu_state.instruction.length();
      for (size_t i = 0; i < instruction_padding; i++) {
        trace_file_ << " ";
      }

      trace_file_ << "A:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(cpu_state.registers.A) << " ";
      trace_file_ << "X:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(cpu_state.registers.X) << " ";
      trace_file_ << "Y:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(cpu_state.registers.Y) << " ";
      trace_file_ << "P:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(cpu_state.registers.SR) << " ";
      trace_file_ << "SP:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(cpu_state.registers.SP) << " ";

      const auto ppu_state = ppu_.get_state();

      trace_file_ << "PPU:";

      trace_file_ << std::dec << std::right << std::setw(3) << std::setfill(' ') << ppu_state.cycle << ",";

      trace_file_ << std::dec << std::right << std::setw(3) << std::setfill(' ') << ppu_state.scanline << " ";

      trace_file_ << "CYC:" << std::dec << cpu_state.cycles;

      trace_file_ << std::endl;
    }
  }

  auto trace_done() -> void {
    if (trace_file_.is_open()) {
      trace_file_.close();
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

  memory_sram sram_{};

  memory_ram ram_{};

  std::ofstream trace_file_{};

  std::unique_ptr<controller::controller> controller_one_{};

  std::unique_ptr<controller::controller> controller_two_{};
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

  auto trace(const char *trace_file) -> void {
    nes_.pimpl_->trace(trace_file);
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

auto debugger::trace(const char *trace_file) -> void {
  impl_->trace(trace_file);
}
