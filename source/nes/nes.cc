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
#include "ppu.hh"

using namespace jones;

template <typename C>
class memory_mappable_component : public memory_mappable {
public:
  memory_mappable_component(C &component, uint16_t start_address, uint16_t end_address)
      : component_(component), start_address_(start_address), end_address_(end_address) {}

  uint16_t start_address() override {
    return start_address_;
  }

  uint16_t end_address() override {
    return end_address_;
  }

  uint8_t read(const uint16_t address) override {
    return component_.read(address);
  }

  void write(const uint16_t address, const uint8_t data) override {
    component_.write(address, data);
  }

private:
  C &component_;
  const uint16_t start_address_;
  const uint32_t end_address_;
};

class memory_sram {
public:
  memory_sram() : sram_(sram_size, 0) {}

  ~memory_sram() = default;

  uint8_t read(const uint16_t address) {
    return sram_[address];
  }

  void write(const uint16_t address, const uint8_t data) {
    sram_[address] = data;
  }

private:
  static constexpr size_t sram_size = 0x2000U;

  std::vector<uint8_t> sram_;
};

class memory_ram {
public:
  memory_ram() : ram_(ram_size, 0) {}

  ~memory_ram() = default;

  uint8_t read(const uint16_t address) {
    return ram_[address];
  }

  void write(const uint16_t address, const uint8_t data) {
    ram_[address] = data;
  }

private:
  static constexpr size_t ram_size = 0x2000U;

  std::vector<uint8_t> ram_;
};

class nes::impl {
public:
  impl() : memory_(), apu_(memory_), cpu_(memory_), ppu_(memory_), cartridge_(), sram_(), ram_(), trace_file_(nullptr) {
    memory_.map(std::make_unique<memory_mappable_component<memory_ram>>(ram_, 0x0000, 0x1FFF));
    memory_.map(std::make_unique<memory_mappable_component<ppu>>(ppu_, 0x2000, 0x3FFF));
    memory_.map(std::make_unique<memory_mappable_component<apu>>(apu_, 0x4000, 0x4017));
    memory_.map(std::make_unique<memory_mappable_component<cpu>>(cpu_, 0x4000, 0x4017));
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
      cpu_.step();
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

private:
  void initialize_components() {
    ppu_.initialize();
    cpu_.initialize();
    apu_.initialize();
  }

  void trace_step() {
    if (trace_file_.is_open()) {
      const auto cpu_state = cpu_.get_state();

      trace_file_ << std::hex << std::uppercase << cpu_state.registers.PC << "  ";

      for (const auto &i : cpu_state.instruction_bytes) {
        trace_file_ << std::left << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(i) << " ";
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

      trace_file_ << "PPU:" << std::dec << std::right << std::setw(3) << std::setfill(' ') << ppu_state.cycles << ",  0 ";

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
  ppu ppu_;
  cartridge cartridge_;
  memory_sram sram_;
  memory_ram ram_;

  std::ofstream trace_file_;
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
