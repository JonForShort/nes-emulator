//
// MIT License
//
// Copyright 2017-2019
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

#include "cpu.hh"
#include "decode.hh"
#include "instruction.hh"
#include "interrupts.hh"
#include "memory.hh"
#include "opcode.hh"
#include "registers.hh"
#include "status_register.hh"

using namespace jones;

class cpu::impl final {
public:
  explicit impl(const memory &memory)
      : memory_(memory),
        status_register_(),
        registers_(),
        interrupts_(),
        skipped_cycles_(0),
        cycles_(0) {}

  void initialize() {
    reset();
  }

  void step() {
    cycles_++;
  }

  void reset() {
    status_register_.set(status_flag::I);
    status_register_.set(status_flag::B1);
    status_register_.set(status_flag::B2);

    registers_.set(register_type::AC, 0x00);
    registers_.set(register_type::X, 0x00);
    registers_.set(register_type::Y, 0x00);
    registers_.set(register_type::SP, 0xFD);

    interrupts_.set_state(interrupt_type::RESET, true);
    interrupts_.set_state(interrupt_type::BRK, false);
    interrupts_.set_state(interrupt_type::IRQ, false);
    interrupts_.set_state(interrupt_type::NMI, false);

    // frame irq is enabled
    memory_.write(0x4017, 0x00);

    // all channels disabled
    memory_.write(0x4015, 0x00);

    // setting to initial pc address
    const auto reset_vector = interrupts_.get_vector(interrupt_type::RESET);
    const auto reset_routine = memory_.read(reset_vector);
    registers_.set(register_type::PC, reset_routine);
  }

  void run() {
  }

  uint8_t read(const uint16_t address) const {
    boost::ignore_unused(address);
    return 0;
  }

  void write(const uint16_t address, uint8_t data) {
    boost::ignore_unused(address);
    boost::ignore_unused(data);
  }

  cpu_state get_state() const {
    return cpu_state();
  }

private:
  void interrupt(const interrupt_type type) {
    if (status_register_.is_set(status_flag::I)) {
      return;
    }

    const auto pc = registers_.get(register_type::PC);
    push(pc);

    uint8_t php[] = {0x08};
    execute(decode::decode(php, sizeof(php)));

    const auto interrupt_vector = interrupts_.get_vector(type);
    registers_.set(register_type::PC, interrupt_vector);

    status_register_.set(status_flag::I);

    cycles_ += 7;
  }

  void push(const uint8_t value) {
    const auto stack_pointer = registers_.get(register_type::SP);
    memory_.write(0x100U | stack_pointer, value);
    registers_.set(register_type::SP, stack_pointer - 1);
  }

  void push(const uint16_t value) {
    const uint8_t high_byte = (value >> 8U);
    const uint8_t low_byte = (value & 0xFFU);
    push(high_byte);
    push(low_byte);
  }

  uint8_t pull() {
    const auto stack_pointer = registers_.get(register_type::SP);
    registers_.set(register_type::SP, stack_pointer + 1);
    return memory_.read(0x100U | stack_pointer);
  }

  void execute(const decode::instruction &instruction) {
    switch (instruction.decoded_opcode.type) {
    case opcode_type::PHP: {
      const uint8_t flags = 0x10U | status_register_.get();
      push(flags);
      break;
    }
    case opcode_type::PLP: {
      const uint8_t flags = (pull() & 0xEFU) | 0x20U;
      status_register_.set(flags);
      break;
    }
    default: {
      break;
    }
    }
  }

private:
  static constexpr int ram_size = 2048;

  const memory &memory_;

  status_register status_register_;

  registers registers_;

  interrupts interrupts_;

  uint16_t skipped_cycles_;

  uint64_t cycles_;
};

cpu::cpu(const memory &memory) : impl_(new impl(memory)) {}

cpu::~cpu() = default;

void cpu::step() { impl_->step(); }

void cpu::reset() { impl_->reset(); }

void cpu::run() { impl_->run(); }

uint8_t cpu::read(uint16_t address) {
  return impl_->read(address);
}

void cpu::write(uint16_t address, uint8_t data) {
  impl_->write(address, data);
}

cpu_state cpu::get_state() {
  return impl_->get_state();
}

void cpu::initialize() {
  impl_->initialize();
}
