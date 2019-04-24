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
#include "disassemble.hh"
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
    const auto decoded = decode();
    execute(decoded);
  }

  void reset() {
    status_register_.set(status_flag::I);
    status_register_.set(status_flag::B2);

    registers_.set(register_type::AC, 0x00);
    registers_.set(register_type::X, 0x00);
    registers_.set(register_type::Y, 0x00);
    registers_.set(register_type::SP, 0xFD);
    registers_.set(register_type::SR, status_register_.get());

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
    const auto reset_routine = (memory_.read(reset_vector) | (memory_.read(reset_vector + 1) << 8)) - 4;
    registers_.set(register_type::PC, reset_routine);

    // cycles for the cost of reset interrupt
    cycles_ += 7;
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
    auto decoded = decode();
    auto encoded_bytes = std::vector<uint8_t>(&decoded.encoded[0], &decoded.encoded[0] + decoded.encoded_length_in_bytes);
    auto disassembled = disassemble::disassemble(&encoded_bytes[0], encoded_bytes.size());

    cpu_state current_cpu_state;
    current_cpu_state.instruction = disassembled.instructions[0].opcode + disassembled.instructions[0].operand;
    current_cpu_state.instruction_bytes = encoded_bytes;
    current_cpu_state.cycles = cycles_;
    current_cpu_state.registers.PC = registers_.get(register_type::PC);
    current_cpu_state.registers.SP = registers_.get(register_type::SP);
    current_cpu_state.registers.A = registers_.get(register_type::AC);
    current_cpu_state.registers.X = registers_.get(register_type::X);
    current_cpu_state.registers.Y = registers_.get(register_type::Y);
    current_cpu_state.registers.SR = registers_.get(register_type::SR);

    return current_cpu_state;
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

  decode::instruction decode() const {
    const uint16_t pc = registers_.get(register_type::PC);
    std::vector<uint8_t> bytes;
    for (size_t pc_offset = 0; pc_offset < decode::max_length_in_bytes; pc_offset++) {
      const auto byte = memory_.read(pc + pc_offset);
      bytes.push_back(byte);
      const auto decoded = decode::decode(&bytes[0], bytes.size());
      if (decoded.decoded_result == decode::result::SUCCESS) {
        return decoded;
      }
    }
    BOOST_STATIC_ASSERT("unexpected decode operation; instruction length too big");
    return decode::instruction();
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
    case opcode_type::JMP: {
      const uint16_t address = std::get<uint16_t>(instruction.decoded_operand.value);
      registers_.set(register_type::PC, address);
      break;
    }
    case opcode_type::LDX: {
      const auto operand = std::get<uint8_t>(instruction.decoded_operand.value);
      const auto operand_type = instruction.decoded_operand.type;
      const auto addressing_mode_type = instruction.decoded_addressing_mode;
      execute_ldx(operand, operand_type, addressing_mode_type);
      break;
    }
    default: {
      break;
    }
    }
  }

  void execute_ldx(const uint8_t operand, const operand_type type, const addressing_mode_type address_mode) {
    update_status_flags(operand);
    switch (type) {
    case operand_type::IMMEDIATE: {
      registers_.set(register_type::X, operand);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (address_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        const auto memory = memory_.read(operand);
        registers_.set(register_type::X, memory);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_X: {
        const auto x_register = registers_.get(register_type::X);
        const auto memory = memory_.read(static_cast<uint16_t>(operand + x_register) & (0xFFU));
        registers_.set(register_type::X, memory);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        const auto memory = memory_.read(operand);
        registers_.set(register_type::X, memory);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_X: {
        const uint8_t x_register = registers_.get(register_type::X);
        const uint16_t memory = memory_.read(operand + x_register);
        registers_.set(register_type::X, memory);
        cycles_ += 4;
        //
        // Check for additional cycle caused by carry.
        //
        if (((memory & 0xFFU) + x_register) < x_register) {
          cycles_++;
        }
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LDX");
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LDX");
      break;
    }
    registers_.increment_by(register_type::PC, sizeof(uint16_t));
  }

  void update_status_flags(const uint8_t binary) {
    const auto bits = std::bitset<8>(binary);
    bits.test(7) ? status_register_.set(status_flag::N) : status_register_.clear(status_flag::N);
    bits == 0 ? status_register_.set(status_flag::Z) : status_register_.clear(status_flag::Z);
    registers_.set(register_type::SR, status_register_.get());
  }

private:
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
