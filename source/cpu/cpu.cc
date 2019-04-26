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
#include <boost/assert.hpp>
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
        cycles_(0) {}

  void initialize() {
    reset();
  }

  void step() {
    const auto fetched = fetch();
    const auto decoded = decode(fetched);
    if (decoded.decoded_result == decode::result::SUCCESS) {
      registers_.increment_by(register_type::PC, decoded.encoded_length_in_bytes);
      execute(decoded);
    } else {
      BOOST_STATIC_ASSERT("unable to step cpu; decoded invalid instruction");
      interrupt(interrupt_type::BRK);
    }
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
    const auto fetched = fetch();
    auto decoded = decode(fetched);
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

  decode::instruction decode(const std::vector<uint8_t> &bytes) const {
    BOOST_ASSERT_MSG(!bytes.empty(), "unexpected decode operation; bytes is empty");
    return decode::decode(const_cast<uint8_t *>(&bytes[0]), bytes.size());
  }

  std::vector<uint8_t> fetch() const {
    const uint16_t pc = registers_.get(register_type::PC);
    uint8_t byte = memory_.read(pc);
    const auto decoded = decode::decode(&byte, sizeof(byte));
    std::vector<uint8_t> result;
    switch (decoded.decoded_result) {
    case decode::result::SUCCESS: {
      result.push_back(byte);
      return result;
    }
    case decode::result::ERROR_REQUIRES_ONE_BYTE: {
      result.push_back(byte);
      return result;
    }
    case decode::result::ERROR_REQUIRES_TWO_BYTES: {
      result.push_back(byte);
      result.push_back(memory_.read(pc + 1));
      break;
    }
    case decode::result::ERROR_REQUIRES_THREE_BYTES: {
      result.push_back(byte);
      result.push_back(memory_.read(pc + 1));
      result.push_back(memory_.read(pc + 2));
      break;
    }
    default: {
      break;
    }
    }
    return result;
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
      execute_jmp(instruction);
      break;
    }
    case opcode_type::LDX: {
      execute_ldx(instruction);
      break;
    }
    case opcode_type::STX: {
      execute_stx(instruction);
      break;
    }
    case opcode_type::STY: {
      execute_sty(instruction);
      break;
    }
    case opcode_type::NOP: {
      execute_nop(instruction);
      break;
    }
    case opcode_type::JSR: {
      execute_jsr(instruction);
      break;
    }
    default: {
      break;
    }
    }
  }

  void execute_jsr(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ABSOLUTE: {
      const uint16_t address = std::get<uint16_t>(instruction.decoded_operand.value);
      const uint16_t next_address = registers_.get(register_type::PC);
      push(next_address);
      registers_.set(register_type::PC, address);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for JSR");
      break;
    }
  }

  void execute_nop(const decode::instruction &instruction) {
    boost::ignore_unused(instruction);
    // nothing to do for instruction.
    cycles_ += 2;
  }

  void execute_sty(const decode::instruction &instruction) {
    const uint8_t register_y = registers_.get(register_type::Y);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      const uint8_t operand = std::get<uint8_t>(instruction.decoded_operand.value);
      memory_.write(operand, register_y);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const uint8_t operand = std::get<uint8_t>(instruction.decoded_operand.value);
      const uint8_t register_x = registers_.get(register_type::X);
      memory_.write(operand + register_x, register_y);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const uint16_t operand = std::get<uint16_t>(instruction.decoded_operand.value);
      memory_.write(operand, register_y);
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STX");
      break;
    }
  }

  void execute_stx(const decode::instruction &instruction) {
    const uint8_t register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      const uint8_t operand = std::get<uint8_t>(instruction.decoded_operand.value);
      memory_.write(operand, register_x);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_Y: {
      const uint8_t operand = std::get<uint8_t>(instruction.decoded_operand.value);
      const uint8_t register_y = registers_.get(register_type::Y);
      memory_.write(operand + register_y, register_x);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const uint16_t operand = std::get<uint16_t>(instruction.decoded_operand.value);
      memory_.write(operand, register_x);
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STX");
      break;
    }
  }

  void execute_jmp(const decode::instruction &instruction) {
    const uint16_t address = std::get<uint16_t>(instruction.decoded_operand.value);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ABSOLUTE: {
      registers_.set(register_type::PC, address);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::INDIRECT: {
      const uint16_t address_page = address & (0xFF00U);
      const auto indirect_address =
          (memory_.read(address)) |
          (memory_.read(address_page | ((address + 1) & 0xFFU)) << 8);
      registers_.set(register_type::PC, indirect_address);
      cycles_ += 5;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for JMP");
      break;
    }
  }

  void execute_ldx(const decode::instruction &instruction) {
    const auto operand = std::get<uint8_t>(instruction.decoded_operand.value);
    const auto operand_type = instruction.decoded_operand.type;
    const auto addressing_mode_type = instruction.decoded_addressing_mode;
    update_status_flags(operand);
    switch (operand_type) {
    case operand_type::IMMEDIATE: {
      registers_.set(register_type::X, operand);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (addressing_mode_type) {
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
