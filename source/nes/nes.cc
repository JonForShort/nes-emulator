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
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>

#include "apu.hh"
#include "cartridge.hh"
#include "cpu.hh"
#include "memory.hh"
#include "nes.hh"
#include "nes_components.hh"
#include "nes_controller.hh"
#include "nes_screen.hh"
#include "ppu.hh"

using namespace jones;

class nes::impl {
public:
  impl()
      : memory_(),
        apu_(memory_),
        cpu_(memory_),
        ppu_(memory_, std::make_unique<nes_screen>()),
        cartridge_(),
        sram_(),
        ram_(),
        trace_file_(),
        screen_(),
        controller_one_(std::make_unique<nes_controller>(memory_)),
        controller_two_(std::make_unique<nes_controller>(memory_)) {
    auto mapped_controller_one = mapped_nes_controller(controller_one_.get());
    auto mapped_controller_two = mapped_nes_controller(controller_two_.get());
    memory_.map(std::make_unique<memory_mappable_component<memory_ram>>(ram_, 0x0000, 0x1FFF));
    memory_.map(std::make_unique<memory_mappable_component<ppu::ppu>>(ppu_, 0x2000, 0x3FFF));
    memory_.map(std::make_unique<memory_mappable_component<apu>>(apu_, 0x4000, 0x4015));
    memory_.map(std::make_unique<memory_mappable_component<cpu>>(cpu_, 0x4000, 0x4015));
    memory_.map(std::make_unique<memory_mappable_component<mapped_nes_controller>>(mapped_controller_one, 0x4016, 0x4016));
    memory_.map(std::make_unique<memory_mappable_component<mapped_nes_controller>>(mapped_controller_two, 0x4017, 0x4017));
    memory_.map(std::make_unique<memory_mappable_component<apu>>(apu_, 0x4018, 0x401F));
    memory_.map(std::make_unique<memory_mappable_component<cpu>>(cpu_, 0x4018, 0x401F));
    memory_.map(std::make_unique<memory_mappable_component<memory_sram>>(sram_, 0x6000, 0x7FFF));
    memory_.map(std::make_unique<memory_mappable_component<cartridge>>(cartridge_, 0x8000, 0xFFFF));
  }

  ~impl() = default;

  bool load(const char *rom_path) {
    if (boost::filesystem::exists(rom_path)) {
      const auto loaded_cartridge = cartridge_.attach(rom_path);
      if (loaded_cartridge) {
        initialize_components();
        return true;
      }
    }
    return false;
  }

  void run(const size_t step_limit) {
    trace_step();
    for (size_t step_count = 0; step_count < step_limit || step_limit == 0; step_count++) {
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
  }

  void reset() {
    initialize_components();
  }

  void trace(const char *trace_file) {
    trace_file_ = std::ofstream{trace_file};
  }

  controller::controller_ptr controller_one() {
    return controller_one_.get();
  }

  controller::controller_ptr controller_two() {
    return controller_two_.get();
  }

  void attach_screen(std::unique_ptr<screen::screen> screen) {
    screen_ = std::move(screen);
  }

private:
  void initialize_components() {
    ppu_.initialize();
    cpu_.initialize();
    apu_.initialize();
  }

  void trace_step() {
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

      trace_file_ << std::dec << std::right << std::setw(3) << std::setfill(' ') << ppu_state.cycles << ",";

      trace_file_ << std::dec << std::right << std::setw(3) << std::setfill(' ') << ppu_state.frames << " ";

      trace_file_ << "CYC:" << std::dec << cpu_state.cycles;

      trace_file_ << std::endl;
    }
  }

  void trace_done() {
    if (trace_file_.is_open()) {
      trace_file_.close();
    }
  }

private:
  memory memory_;

  apu apu_;

  cpu cpu_;

  ppu::ppu ppu_;

  cartridge cartridge_;

  memory_sram sram_;

  memory_ram ram_;

  std::ofstream trace_file_;

  std::unique_ptr<screen::screen> screen_;

  std::unique_ptr<controller::controller> controller_one_;

  std::unique_ptr<controller::controller> controller_two_;
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

void nes::reset() {
  pimpl_->reset();
}

void nes::trace(const char *trace_file) {
  pimpl_->trace(trace_file);
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
