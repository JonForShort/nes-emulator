//
// MIT License
//
// Copyright 2017-2021
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
#include <boost/format.hpp>
#include <coroutine>
#include <cppcoro/generator.hpp>

#include "cpu.hh"
#include "decode.hh"
#include "disassemble_internal.hh"
#include "instruction.hh"
#include "interrupts.hh"
#include "memory.hh"
#include "opcode.hh"
#include "registers.hh"
#include "status_register.hh"

using namespace jones;
using cpu_step = cppcoro::generator<uint8_t>;

class cpu::impl final {
public:
  explicit impl(memory const &memory) : memory_(memory) {}

  auto initialize() -> void {
    is_running_ = true;
    reset();
  }

  auto uninitialize() -> void {
    is_running_ = false;
  }

  auto step() -> uint8_t {
    if (idle_cycles_ > 0) {
      static auto step_idle = step_idle_.begin();
      step_idle++;
      return *step_idle;
    } else if (is_interrupt_pending()) {
      static auto step_interrupt = step_interrupt_.begin();
      step_interrupt++;
      return *step_interrupt;
    } else {
      static auto step_execute = step_execute_.begin();
      step_execute++;
      return *step_execute;
    }
  }

  auto reset() -> void {
    status_register_.set(status_flag::I);

    registers_.set(register_type::AC, 0x00);
    registers_.set(register_type::X, 0x00);
    registers_.set(register_type::Y, 0x00);
    registers_.set(register_type::SP, 0xFD);
    registers_.set(register_type::SR, status_register_.get());

    interrupts_.set_state(interrupt_type::RESET, false);
    interrupts_.set_state(interrupt_type::BRK, false);
    interrupts_.set_state(interrupt_type::IRQ, false);
    interrupts_.set_state(interrupt_type::NMI, false);

    memory_.write(0x4017, 0x00);
    memory_.write(0x4015, 0x00);

    auto const interrupt_vector = interrupts_.get_vector(interrupt_type::RESET);
    auto const interrupt_routine = memory_.read_word(interrupt_vector);
    registers_.set(register_type::PC, interrupt_routine);
  }

  [[nodiscard]] auto peek(uint16_t const address) const -> uint8_t {
    return memory_.peek(address);
  }

  [[nodiscard]] auto read(uint16_t const address) const -> uint8_t {
    return memory_.read(address);
  }

  auto write(uint16_t const address, uint8_t const data) -> void {
    memory_.write(address, data);
  }

  [[nodiscard]] auto get_state() const -> cpu_state {

    class listener : public disassemble::disassemble_listener {
    public:
      explicit listener(registers const &registers) : registers_(registers) {}

      ~listener() override = default;

      auto on_decoded(decode::instruction &instruction) -> void override {
        if (instruction.decoded_addressing_mode == addressing_mode_type::RELATIVE) {
          const int8_t address_relative = std::get<uint8_t>(instruction.decoded_operand.value);
          uint16_t const address_absolute = registers_.get(register_type::PC) + address_relative + instruction.encoded_length_in_bytes;
          instruction.decoded_operand.value = static_cast<uint16_t>(address_absolute);
          instruction.decoded_addressing_mode = addressing_mode_type::ABSOLUTE;
        }
      }

      auto on_disassembled(disassemble::instruction &instruction) -> void override {
        boost::ignore_unused(instruction);
      }

    private:
      registers const &registers_;
    };

    auto fetched = fetch();
    auto const disassembled = disassemble::disassemble(&fetched[0], fetched.size(), std::make_unique<listener>(registers_));

    cpu_state current_cpu_state;
    current_cpu_state.instruction.instruction = disassembled.instructions[0].opcode + disassembled.instructions[0].operand;
    current_cpu_state.instruction.instruction_bytes = fetched;
    current_cpu_state.cycles = cycles_;
    current_cpu_state.registers.PC = registers_.get(register_type::PC);
    current_cpu_state.registers.SP = registers_.get(register_type::SP);
    current_cpu_state.registers.A = registers_.get(register_type::AC);
    current_cpu_state.registers.X = registers_.get(register_type::X);
    current_cpu_state.registers.Y = registers_.get(register_type::Y);
    current_cpu_state.registers.SR = registers_.get(register_type::SR);

    auto instruction_binary = disassembled.instructions[0].binary;
    auto const decoded = decode::decode(instruction_binary.data(), instruction_binary.size());

    switch (decoded.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = false;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = true;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_Y: {
      auto const address = get_zero_page_y_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = true;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = false;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = true;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = true;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = true;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = true;
      break;
    }
    case addressing_mode_type::RELATIVE: {
      auto const address = get_relative_address(decoded) + decoded.encoded_length_in_bytes;
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = false;
      break;
    }
    case addressing_mode_type::INDIRECT: {
      auto const address = get_indirect_address(decoded);
      current_cpu_state.instruction.memory.address = address;
      current_cpu_state.instruction.memory.value = memory_.peek(address);
      current_cpu_state.instruction.memory.is_indirect = true;
      break;
    }
    default:
      break;
    }

    return current_cpu_state;
  }

  auto set_state(cpu_state const &state) -> void {
    registers_.set(register_type::PC, state.registers.PC);
    registers_.set(register_type::SP, state.registers.SP);
    registers_.set(register_type::AC, state.registers.A);
    registers_.set(register_type::X, state.registers.X);
    registers_.set(register_type::Y, state.registers.Y);
    registers_.set(register_type::SR, state.registers.SR);
  }

  auto interrupt(interrupt_type const type, interrupt_state const state) -> void {
    auto const is_set = state == interrupt_state::SET;
    switch (type) {
    case interrupt_type::RESET:
    case interrupt_type::NMI:
      interrupts_.set_state(type, is_set);
      break;
    case interrupt_type::BRK:
    case interrupt_type::IRQ:
      if (!status_register_.is_set(status_flag::I)) {
        interrupts_.set_state(type, is_set);
      }
      break;
    default:
      break;
    }
  }

  auto idle(uint16_t const cycles) -> void {
    idle_cycles_ += cycles;
  }

private:
  auto is_interrupt_pending() -> bool {
    return interrupts_.get_triggered() != interrupt_type::NONE;
  }

  auto step_idle() -> cpu_step {
    co_yield 0;
    while (is_running_) {
      idle_cycles_ -= 1;
      co_yield 1;
    }
  }

  auto step_execute() -> cpu_step {
    co_yield 0;
    while (is_running_) {
      auto const fetched = fetch();
      auto const decoded = decode(fetched);
      if (decoded.decoded_result == decode::result::SUCCESS) {
        registers_.increment_by(register_type::PC, decoded.encoded_length_in_bytes);
        for (auto cycles : execute(decoded)) {
          co_yield cycles;
        }
      } else {
        BOOST_STATIC_ASSERT("unable to step cpu; decoded invalid instruction");
        interrupt(interrupt_type::BRK, interrupt_state::SET);
        co_yield(0);
      }
    }
  }

  auto step_interrupt() -> cpu_step {
    co_yield 0;
    while (is_running_) {
      auto const triggered_interrupt = interrupts_.get_triggered();
      if (triggered_interrupt == interrupt_type::NONE) {
        co_return;
      }
      push_pc();
      push_flags();
      auto const interrupt_vector = interrupts_.get_vector(triggered_interrupt);
      auto const interrupt_routine = memory_.read_word(interrupt_vector);
      registers_.set(register_type::PC, interrupt_routine);
      status_register_.set(status_flag::I);
      registers_.set(register_type::SR, status_register_.get());
      interrupts_.set_state(triggered_interrupt, false);
      co_yield 7;
    }
  }

  auto push(uint8_t const value) -> void {
    memory_.write(get_stack_pointer(), value);
    registers_.decrement(register_type::SP);
  }

  auto pull() -> uint8_t {
    registers_.increment(register_type::SP);
    return memory_.read(get_stack_pointer());
  }

  auto push_flags() -> void {
    const uint8_t flags = 0x10U | status_register_.get();
    push(flags);
  }

  auto pull_flags() -> void {
    uint8_t const flags = (pull() & 0xEFU) | 0x20U;
    status_register_.set(flags);
    registers_.set(register_type::SR, status_register_.get());
  }

  auto push_pc() -> void {
    uint16_t const pc = registers_.get(register_type::PC);
    push(pc >> 8U);
    push(pc);
  }

  auto pull_pc() -> void {
    uint16_t const low_byte = pull();
    uint16_t const high_byte = pull() << 8U;
    registers_.set(register_type::PC, (low_byte | high_byte));
  }

  [[nodiscard]] auto get_stack_pointer() const -> uint16_t {
    return stack_pointer_base | registers_.get(register_type::SP);
  }

  [[nodiscard]] auto decode(std::vector<uint8_t> const &bytes) const -> decode::instruction {
    BOOST_ASSERT_MSG(!bytes.empty(), "unexpected decode operation; bytes is empty");
    return decode::decode(const_cast<uint8_t *>(&bytes[0]), bytes.size());
  }

  [[nodiscard]] auto fetch() const -> std::vector<uint8_t> {
    uint16_t const pc = registers_.get(register_type::PC);
    uint8_t byte = memory_.read(pc);
    auto const decoded = decode::decode(&byte, sizeof(byte));
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

  auto execute(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_opcode.type) {
    case opcode_type::BRK: {
      interrupt(interrupt_type::BRK, interrupt_state::SET);
      return execute_noop();
    }
    case opcode_type::PHP: {
      return execute_php(instruction);
    }
    case opcode_type::PLP: {
      return execute_plp(instruction);
    }
    case opcode_type::PHA: {
      return execute_pha(instruction);
    }
    case opcode_type::PLA: {
      return execute_pla(instruction);
    }
    case opcode_type::JMP: {
      return execute_jmp(instruction);
    }
    case opcode_type::LDA: {
      return execute_lda(instruction);
    }
    case opcode_type::LDX: {
      return execute_ldx(instruction);
    }
    case opcode_type::LDY: {
      return execute_ldy(instruction);
    }
    case opcode_type::STA: {
      return execute_sta(instruction);
    }
    case opcode_type::STX: {
      return execute_stx(instruction);
    }
    case opcode_type::STY: {
      return execute_sty(instruction);
    }
    case opcode_type::NOP: {
      return execute_nop(instruction);
    }
    case opcode_type::JSR: {
      return execute_jsr(instruction);
    }
    case opcode_type::CLC: {
      return execute_clc(instruction);
    }
    case opcode_type::SEC: {
      return execute_sec(instruction);
    }
    case opcode_type::CLI: {
      return execute_cli(instruction);
    }
    case opcode_type::SEI: {
      return execute_sei(instruction);
    }
    case opcode_type::CLD: {
      return execute_cld(instruction);
    }
    case opcode_type::SED: {
      return execute_sed(instruction);
    }
    case opcode_type::CLV: {
      return execute_clv(instruction);
    }
    case opcode_type::BCS: {
      return execute_bcs(instruction);
    }
    case opcode_type::BCC: {
      return execute_bcc(instruction);
    }
    case opcode_type::BVS: {
      return execute_bvs(instruction);
    }
    case opcode_type::BVC: {
      return execute_bvc(instruction);
    }
    case opcode_type::BEQ: {
      return execute_beq(instruction);
    }
    case opcode_type::BNE: {
      return execute_bne(instruction);
    }
    case opcode_type::BMI: {
      return execute_bmi(instruction);
    }
    case opcode_type::BPL: {
      return execute_bpl(instruction);
    }
    case opcode_type::BIT: {
      return execute_bit(instruction);
    }
    case opcode_type::ADC: {
      return execute_adc(instruction);
    }
    case opcode_type::SBC: {
      return execute_sbc(instruction);
    }
    case opcode_type::AND: {
      return execute_and(instruction);
    }
    case opcode_type::ASL: {
      return execute_asl(instruction);
    }
    case opcode_type::LSR: {
      return execute_lsr(instruction);
    }
    case opcode_type::ROR: {
      return execute_ror(instruction);
    }
    case opcode_type::ROL: {
      return execute_rol(instruction);
    }
    case opcode_type::CMP: {
      return execute_cmp(instruction);
    }
    case opcode_type::CPX: {
      return execute_cpx(instruction);
    }
    case opcode_type::CPY: {
      return execute_cpy(instruction);
    }
    case opcode_type::RTS: {
      return execute_rts(instruction);
    }
    case opcode_type::ORA: {
      return execute_ora(instruction);
    }
    case opcode_type::EOR: {
      return execute_eor(instruction);
    }
    case opcode_type::INY: {
      return execute_iny(instruction);
    }
    case opcode_type::INX: {
      return execute_inx(instruction);
    }
    case opcode_type::INC: {
      return execute_inc(instruction);
    }
    case opcode_type::DEC: {
      return execute_dec(instruction);
    }
    case opcode_type::DEY: {
      return execute_dey(instruction);
    }
    case opcode_type::DEX: {
      return execute_dex(instruction);
    }
    case opcode_type::TAY: {
      return execute_tay(instruction);
    }
    case opcode_type::TYA: {
      return execute_tya(instruction);
    }
    case opcode_type::TAX: {
      return execute_tax(instruction);
    }
    case opcode_type::TXA: {
      return execute_txa(instruction);
    }
    case opcode_type::TSX: {
      return execute_tsx(instruction);
    }
    case opcode_type::TXS: {
      return execute_txs(instruction);
    }
    case opcode_type::RTI: {
      return execute_rti(instruction);
    }
    case opcode_type::LAX: {
      return execute_lax(instruction);
    }
    case opcode_type::SAX: {
      return execute_sax(instruction);
    }
    case opcode_type::DCP: {
      return execute_dcp(instruction);
    }
    case opcode_type::ISC: {
      return execute_isc(instruction);
    }
    case opcode_type::SLO: {
      return execute_slo(instruction);
    }
    case opcode_type::RLA: {
      return execute_rla(instruction);
    }
    case opcode_type::SRE: {
      return execute_sre(instruction);
    }
    case opcode_type::RRA: {
      return execute_rra(instruction);
    }
    default: {
      return execute_noop();
    }
    }
  }

  auto execute_noop() -> cpu_step {
    co_yield(0);
  }

  auto execute_rra(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      co_yield 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      co_yield 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      co_yield 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      co_yield 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute RRA");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_rra(const uint8_t value) -> uint8_t {
    auto const is_carry_set = status_register_.is_set(status_flag::C);
    auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (is_carry_set ? 1U << 7U : 0U);

    auto const ac_register = registers_.get(register_type::AC);
    auto const addition = static_cast<uint16_t>(ac_register + shifted_value + ((value & 0x1U) ? 1 : 0));

    registers_.set(register_type::AC, addition & 0xFFU);

    auto const caused_overflow = (static_cast<uint16_t>(ac_register ^ shifted_value) & 0x80U) == 0 &&
                                 (static_cast<uint16_t>(ac_register ^ registers_.get(register_type::AC)) & 0x80U) != 0;

    update_status_flag_c(addition > 0xFFU);
    update_status_flag_v(caused_overflow);
    update_status_flag_zn(registers_.get(register_type::AC));
    return shifted_value;
  }

  auto execute_sre(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      co_yield 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      co_yield 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      co_yield 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      co_yield 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute SRE");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_sre(const uint8_t value) -> uint8_t {
    auto const shifted_value = value >> 0x1U;
    auto const ac_register = registers_.get(register_type::AC);
    auto const new_ac_register = ac_register ^ shifted_value;
    registers_.set(register_type::AC, new_ac_register);
    update_status_flag_c(value & 0x1U);
    update_status_flag_zn(new_ac_register);
    return shifted_value;
  }

  auto execute_rla(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      co_yield 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      co_yield 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      co_yield 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      co_yield 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute RLA");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_rla(const uint8_t value) -> uint8_t {
    auto const has_carry = status_register_.is_set(status_flag::C);
    auto const shifted_value = (value << 1) | (has_carry ? 1 : 0);
    auto const ac_register = registers_.get(register_type::AC);
    auto const new_ac_register = ac_register & shifted_value;
    registers_.set(register_type::AC, new_ac_register & 0xFF);
    update_status_flag_c((value >> 7) & 0x1U);
    update_status_flag_zn(new_ac_register);
    return shifted_value;
  }

  auto execute_slo(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      execute_slo(address);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      execute_slo(address);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      execute_slo(address);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      execute_slo(address);
      co_yield 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      execute_slo(address);
      co_yield 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      execute_slo(address);
      co_yield 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      execute_slo(address);
      co_yield 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute SLO");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_slo(uint16_t const address) -> void {
    auto const value = memory_.read(address);
    auto const shifted_value = value << 1U;
    memory_.write(address, shifted_value);
    update_status_flag_c(std::bitset<8>(value).test(7));

    auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
    registers_.set(register_type::AC, ora_value);
    update_status_flag_zn(ora_value);
  }

  auto execute_isc(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      co_yield 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      co_yield 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      co_yield 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      co_yield 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute ISC");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_dcp(decode::instruction const &instruction) -> cpu_step {
    auto const register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      co_yield 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      co_yield 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      co_yield 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      co_yield 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for DCP");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_sax(decode::instruction const &instruction) -> cpu_step {
    auto const register_ac = registers_.get(register_type::AC);
    auto const register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_ac & register_x);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_Y: {
      auto const address = get_zero_page_y_address(instruction);
      memory_.write(address, register_ac & register_x);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_ac & register_x);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      memory_.write(address, register_ac & register_x);
      co_yield 5;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      memory_.write(address, register_ac & register_x);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      memory_.write(address, register_ac & register_x);
      co_yield 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for SAX");
      co_yield 0;
      break;
    }
  }

  auto execute_lax(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, value);
      registers_.set(register_type::X, value);
      co_yield 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        co_yield 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_Y: {
        auto const address = get_zero_page_y_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        auto const address = get_absolute_y_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        co_yield 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::INDEXED_INDIRECT: {
        auto const address = get_indexed_indirect_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        co_yield 6;
        break;
      }
      case addressing_mode_type::INDIRECT_INDEXED: {
        bool is_page_crossed = false;
        auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        co_yield 5 + (is_page_crossed ? 1 : 0);
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LAX");
        co_yield 0;
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LAX");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  auto execute_rti(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      pull_flags();
      pull_pc();
      co_yield 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for RTI");
      co_yield 0;
      break;
    }
  }

  auto execute_transfer(decode::instruction const &instruction, const register_type source, const register_type destination, const bool should_update_status = true) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      registers_.set(destination, registers_.get(source));
      co_yield 2;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute transfer");
      co_yield 0;
      break;
    }
    if (should_update_status) {
      update_status_flag_zn(registers_.get(destination));
    }
  }

  auto execute_tay(decode::instruction const &instruction) -> cpu_step {
    return execute_transfer(instruction, register_type::AC, register_type::Y);
  }

  auto execute_tya(decode::instruction const &instruction) -> cpu_step {
    return execute_transfer(instruction, register_type::Y, register_type::AC);
  }

  auto execute_tax(decode::instruction const &instruction) -> cpu_step {
    return execute_transfer(instruction, register_type::AC, register_type::X);
  }

  auto execute_txa(decode::instruction const &instruction) -> cpu_step {
    return execute_transfer(instruction, register_type::X, register_type::AC);
  }

  auto execute_tsx(decode::instruction const &instruction) -> cpu_step {
    return execute_transfer(instruction, register_type::SP, register_type::X);
  }

  auto execute_txs(decode::instruction const &instruction) -> cpu_step {
    return execute_transfer(instruction, register_type::X, register_type::SP, false);
  }

  auto execute_decrement(decode::instruction const &instruction, const register_type type) -> cpu_step {
    auto const value = registers_.get(type);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      registers_.set(type, value - 1);
      co_yield 2;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute decrement");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(type));
  }

  auto execute_dey(decode::instruction const &instruction) -> cpu_step {
    return execute_decrement(instruction, register_type::Y);
  }

  auto execute_dex(decode::instruction const &instruction) -> cpu_step {
    return execute_decrement(instruction, register_type::X);
  }

  auto execute_increment(decode::instruction const &instruction, const register_type type) -> cpu_step {
    auto const value = registers_.get(type);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      registers_.set(type, value + 1);
      co_yield 2;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute increment");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(type));
  }

  auto execute_iny(decode::instruction const &instruction) -> cpu_step {
    return execute_increment(instruction, register_type::Y);
  }

  auto execute_inx(decode::instruction const &instruction) -> cpu_step {
    return execute_increment(instruction, register_type::X);
  }

  auto execute_inc(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for INC");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_dec(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      co_yield 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for DEC");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_eor(decode::instruction const &instruction) -> cpu_step {
    auto const ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      co_yield 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for EOR");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  auto execute_ora(decode::instruction const &instruction) -> cpu_step {
    auto const ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      co_yield 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ORA");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  auto execute_pha(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      auto const value = registers_.get(register_type::AC);
      push(value);
      co_yield 3;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PHA");
      co_yield 0;
      break;
    }
  }

  auto execute_pla(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      auto const value = pull();
      registers_.set(register_type::AC, value);
      update_status_flag_zn(value);
      co_yield 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PLA");
      co_yield 0;
      break;
    }
  }

  auto execute_php(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      push_flags();
      co_yield 3;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PHP");
      co_yield 0;
      break;
    }
  }

  auto execute_plp(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      pull_flags();
      co_yield 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PLP");
      co_yield 0;
      break;
    }
  }

  auto execute_cpy(decode::instruction const &instruction) -> cpu_step {
    auto const register_y = registers_.get(register_type::Y);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      co_yield 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for CPY");
      co_yield 0;
      break;
    }
  }

  auto execute_cpx(decode::instruction const &instruction) -> cpu_step {
    auto const register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      co_yield 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for CPX");
      co_yield 0;
      break;
    }
  }

  auto execute_cmp(decode::instruction const &instruction) -> cpu_step {
    auto const register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      co_yield 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for CMP");
      co_yield 0;
      break;
    }
  }

  auto execute_lsr(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const register_ac = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const value = register_ac >> 0x1U;
      registers_.set(register_type::AC, value);
      update_status_flag_c(register_ac & 0x1U);
      update_status_flag_zn(value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for LSR");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_ror(decode::instruction const &instruction) -> cpu_step {
    auto const has_carry = status_register_.is_set(status_flag::C);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const value = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      co_yield 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for ROR");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_rol(decode::instruction const &instruction) -> cpu_step {
    auto const has_carry = status_register_.is_set(status_flag::C);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const value = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      co_yield 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for ROL");
      co_yield 0;
      break;
    }
    }
  }

  auto execute_asl(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const value = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const shifted_value = value << 1U;
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      co_yield 7;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ASL");
      co_yield 0;
      break;
    }
  }

  auto execute_sbc(decode::instruction const &instruction) -> cpu_step {
    uint16_t value = 0;
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      value = get_immediate(instruction);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      value = memory_.read(address);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      value = memory_.read(address);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      value = memory_.read(address);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      value = memory_.read(address);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      value = memory_.read(address);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      value = memory_.read(address);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      value = memory_.read(address);
      co_yield 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ADC");
      break;
    }
    execute_sbc(value);
  }

  auto execute_sbc(const uint8_t value) -> void {
    auto const ac_register = registers_.get(register_type::AC);
    auto const is_carry_set = status_register_.is_set(status_flag::C);

    auto const subtraction = static_cast<int>(ac_register - value - (1 - (is_carry_set ? 1 : 0)));
    registers_.set(register_type::AC, static_cast<uint16_t>(subtraction) & 0xFFU);

    auto const caused_overflow = (static_cast<uint16_t>(ac_register ^ value) & 0x80U) != 0 &&
                                 (static_cast<uint16_t>(ac_register ^ registers_.get(register_type::AC)) & 0x80U) != 0;

    status_register_.set(status_flag::C, subtraction >= 0);
    status_register_.set(status_flag::V, caused_overflow);

    update_status_flag_zn(registers_.get(register_type::AC));
  }

  auto execute_adc(decode::instruction const &instruction) -> cpu_step {
    auto const ac_register = registers_.get(register_type::AC);
    auto const is_carry_set = status_register_.is_set(status_flag::C);
    uint16_t value = 0;
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      value = get_immediate(instruction);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      value = memory_.read(address);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      value = memory_.read(address);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      value = memory_.read(address);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      value = memory_.read(address);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      value = memory_.read(address);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      value = memory_.read(address);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      value = memory_.read(address);
      co_yield 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ADC");
      co_yield 0;
      break;
    }
    auto const addition = static_cast<uint16_t>(ac_register + value + (is_carry_set ? 1 : 0));
    registers_.set(register_type::AC, addition);

    auto const caused_overflow = (static_cast<uint16_t>(ac_register ^ value) & 0x80U) == 0 &&
                                 (static_cast<uint16_t>(ac_register ^ registers_.get(register_type::AC)) & 0x80U) != 0;

    status_register_.set(status_flag::C, addition > 0xFFU);
    status_register_.set(status_flag::V, caused_overflow);
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  auto execute_and(decode::instruction const &instruction) -> cpu_step {
    auto const ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      co_yield 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for AND");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  auto execute_bit(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const bits = std::bitset<8>(value);
      bits.test(6) ? status_register_.set(status_flag::V) : status_register_.clear(status_flag::V);
      bits.test(7) ? status_register_.set(status_flag::N) : status_register_.clear(status_flag::N);
      update_status_flag_z(value & registers_.get(register_type::AC));
      co_yield 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const bits = std::bitset<8>(value);
      bits.test(6) ? status_register_.set(status_flag::V) : status_register_.clear(status_flag::V);
      bits.test(7) ? status_register_.set(status_flag::N) : status_register_.clear(status_flag::N);
      update_status_flag_z(value & registers_.get(register_type::AC));
      co_yield 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for BIT");
      co_yield 0;
      break;
    }
  }

  auto execute_bmi(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::N, true);
  }

  auto execute_bpl(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::N, false);
  }

  auto execute_bne(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::Z, false);
  }

  auto execute_beq(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::Z, true);
  }

  auto execute_bvc(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::V, false);
  }

  auto execute_bvs(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::V, true);
  }

  auto execute_bcc(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::C, false);
  }

  auto execute_bcs(decode::instruction const &instruction) -> cpu_step {
    return execute_branch(instruction, status_flag::C, true);
  }

  auto execute_branch(decode::instruction const &instruction, status_flag const flag, bool const is_flag_set) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::RELATIVE: {
      co_yield 2;
      if (status_register_.is_set(flag) == is_flag_set) {
        co_yield 1;
        uint16_t const previous_pc = registers_.get(register_type::PC);
        uint16_t const previous_page = previous_pc & 0xFF00U;
        uint16_t const next_pc = get_relative_address(instruction);
        uint16_t const next_page = next_pc & 0xFF00U;
        if (previous_page != next_page) {
          co_yield 1;
        }
        registers_.set(register_type::PC, next_pc);
      }
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for BRANCH");
      co_yield 0;
      break;
    }
  }

  auto execute_clv(decode::instruction const &instruction) -> cpu_step {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::V);
    registers_.set(register_type::SR, status_register_.get());
    co_yield 2;
  }

  auto execute_cld(decode::instruction const &instruction) -> cpu_step {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::D);
    registers_.set(register_type::SR, status_register_.get());
    co_yield 2;
  }

  auto execute_sed(decode::instruction const &instruction) -> cpu_step {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.set(status_flag::D);
    registers_.set(register_type::SR, status_register_.get());
    co_yield 2;
  }

  auto execute_cli(decode::instruction const &instruction) -> cpu_step {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::I);
    registers_.set(register_type::SR, status_register_.get());
    co_yield 2;
  }

  auto execute_sei(decode::instruction const &instruction) -> cpu_step {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.set(status_flag::I);
    registers_.set(register_type::SR, status_register_.get());
    co_yield 2;
  }

  auto execute_clc(decode::instruction const &instruction) -> cpu_step {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::C);
    registers_.set(register_type::SR, status_register_.get());
    co_yield 2;
  }

  auto execute_sec(decode::instruction const &instruction) -> cpu_step {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.set(status_flag::C);
    registers_.set(register_type::SR, status_register_.get());
    co_yield 2;
  }

  auto execute_jsr(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ABSOLUTE: {
      registers_.decrement(register_type::PC);
      push_pc();
      uint16_t const address = get_absolute_address(instruction);
      registers_.set(register_type::PC, address);
      co_yield 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for JSR");
      co_yield 0;
      break;
    }
  }

  auto execute_rts(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      pull_pc();
      registers_.increment(register_type::PC);
      co_yield 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for RTS");
      co_yield 0;
      break;
    }
  }

  auto execute_nop(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT:
      co_yield 2;
      break;
    case addressing_mode_type::IMMEDIATE:
      co_yield 2;
      break;
    case addressing_mode_type::ZERO_PAGE:
      co_yield 3;
      break;
    case addressing_mode_type::ZERO_PAGE_X:
      co_yield 4;
      break;
    case addressing_mode_type::ABSOLUTE:
      co_yield 4;
      break;
    case addressing_mode_type::ABSOLUTE_X: {
      auto is_page_crossed = false;
      (void)get_absolute_x_address(instruction, is_page_crossed);
      co_yield 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for NOP");
      co_yield 0;
      break;
    }
  }

  auto execute_sty(decode::instruction const &instruction) -> cpu_step {
    auto const register_y = registers_.get(register_type::Y);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_y);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      memory_.write(address, register_y);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_y);
      co_yield 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STY");
      co_yield 0;
      break;
    }
  }

  auto execute_sta(decode::instruction const &instruction) -> cpu_step {
    auto const register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_ac);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      memory_.write(address, register_ac);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_ac);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      memory_.write(address, register_ac);
      co_yield 5;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      memory_.write(address, register_ac);
      co_yield 5;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      memory_.write(address, register_ac);
      co_yield 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      memory_.write(address, register_ac);
      co_yield 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STA");
      co_yield 0;
      break;
    }
  }

  auto execute_stx(decode::instruction const &instruction) -> cpu_step {
    auto const register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_x);
      co_yield 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_Y: {
      auto const address = get_zero_page_y_address(instruction);
      memory_.write(address, register_x);
      co_yield 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_x);
      co_yield 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STX");
      co_yield 0;
      break;
    }
  }

  auto execute_jmp(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      registers_.set(register_type::PC, address);
      co_yield 3;
      break;
    }
    case addressing_mode_type::INDIRECT: {
      auto const address = get_indirect_address(instruction);
      registers_.set(register_type::PC, address);
      co_yield 5;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for JMP");
      co_yield 0;
      break;
    }
  }

  auto execute_lda(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, value);
      co_yield 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        co_yield 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_X: {
        auto const address = get_zero_page_x_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_X: {
        bool is_page_crossed = false;
        auto const address = get_absolute_x_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        co_yield 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        auto const address = get_absolute_y_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        co_yield 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::INDEXED_INDIRECT: {
        auto const address = get_indexed_indirect_address(instruction);
        auto const memory = memory_.read(address);
        registers_.set(register_type::AC, memory);
        co_yield 6;
        break;
      }
      case addressing_mode_type::INDIRECT_INDEXED: {
        bool is_page_crossed = false;
        auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        co_yield 5 + (is_page_crossed ? 1 : 0);
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LDA");
        co_yield 0;
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LDA");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  auto execute_ldy(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::Y, value);
      co_yield 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        co_yield 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_X: {
        auto const address = get_zero_page_x_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_X: {
        bool is_page_crossed = false;
        auto const address = get_absolute_x_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        co_yield 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LDY");
        co_yield 0;
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LDY");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(register_type::Y));
  }

  auto execute_ldx(decode::instruction const &instruction) -> cpu_step {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::X, value);
      co_yield 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        co_yield 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_Y: {
        auto const address = get_zero_page_y_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        co_yield 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        auto const address = get_absolute_y_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        co_yield 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LDX");
        co_yield 0;
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LDX");
      co_yield 0;
      break;
    }
    update_status_flag_zn(registers_.get(register_type::X));
  }

  void update_status_flag_zn(const uint8_t binary) {
    update_status_flag_n(binary);
    update_status_flag_z(binary);
  }

  void update_status_flag_n(const uint8_t binary) {
    auto const bits = std::bitset<8>(binary);
    status_register_.set(status_flag::N, bits.test(7));
    registers_.set(register_type::SR, status_register_.get());
  }

  void update_status_flag_z(const uint8_t binary) {
    auto const bits = std::bitset<8>(binary);
    status_register_.set(status_flag::Z, bits.none());
    registers_.set(register_type::SR, status_register_.get());
  }

  void update_status_flag_v(const bool has_overflow) {
    status_register_.set(status_flag::V, has_overflow);
    registers_.set(register_type::SR, status_register_.get());
  }

  void update_status_flag_c(const bool has_carry) {
    status_register_.set(status_flag::C, has_carry);
    registers_.set(register_type::SR, status_register_.get());
  }

  [[nodiscard]] auto get_zero_page_address(decode::instruction const &instruction) const -> uint16_t {
    return std::get<uint8_t>(instruction.decoded_operand.value) & 0xFFU;
  }

  [[nodiscard]] auto get_zero_page_x_address(decode::instruction const &instruction) const -> uint16_t {
    auto const address = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const x_register = registers_.get(register_type::X);
    return static_cast<uint16_t>(address + x_register) & 0xFFU;
  }

  [[nodiscard]] auto get_zero_page_y_address(decode::instruction const &instruction) const -> uint16_t {
    auto const address = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const y_register = registers_.get(register_type::Y);
    return static_cast<uint16_t>(address + y_register) & 0xFFU;
  }

  [[nodiscard]] auto get_absolute_address(decode::instruction const &instruction) const -> uint16_t {
    return std::get<uint16_t>(instruction.decoded_operand.value);
  }

  [[nodiscard]] auto get_absolute_y_address(decode::instruction const &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_absolute_y_address(instruction, is_page_crossed);
  }

  auto get_absolute_y_address(decode::instruction const &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const address = std::get<uint16_t>(instruction.decoded_operand.value);
    auto const y_register = registers_.get(register_type::Y);
    is_page_crossed = (static_cast<uint16_t>(address + y_register) & 0xFFU) < y_register;
    return address + y_register;
  }

  [[nodiscard]] auto get_absolute_x_address(decode::instruction const &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_absolute_x_address(instruction, is_page_crossed);
  }

  auto get_absolute_x_address(decode::instruction const &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const address = std::get<uint16_t>(instruction.decoded_operand.value);
    auto const x_register = registers_.get(register_type::X);
    is_page_crossed = (static_cast<uint16_t>(address + x_register) & 0xFFU) < x_register;
    return address + x_register;
  }

  [[nodiscard]] auto get_indexed_indirect_address(decode::instruction const &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_indexed_indirect_address(instruction, is_page_crossed);
  }

  auto get_indexed_indirect_address(decode::instruction const &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const operand = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const x_register = registers_.get(register_type::X);
    auto const address = static_cast<uint16_t>(operand + x_register) & 0xFFU;
    is_page_crossed = address < x_register;
    return get_indirect_address(address);
  }

  [[nodiscard]] auto get_indirect_indexed_address(decode::instruction const &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_indirect_indexed_address(instruction, is_page_crossed);
  }

  auto get_indirect_indexed_address(decode::instruction const &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const operand = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const address = get_indirect_address(operand);
    auto const y_register = registers_.get(register_type::Y);
    is_page_crossed = (static_cast<uint16_t>(address + y_register) & 0xFFU) < y_register;
    return address + y_register;
  }

  [[nodiscard]] auto get_indirect_address(decode::instruction const &instruction) const -> uint16_t {
    return get_indirect_address(std::get<uint16_t>(instruction.decoded_operand.value));
  }

  [[nodiscard]] auto get_indirect_address(uint16_t const address) const -> uint16_t {
    auto const address_page = address & 0xFF00U;
    auto const address_high = (address + 1U) & 0xFFU;
    auto const indirect_address_low = memory_.read(address);
    auto const indirect_address_high = memory_.read(address_page | address_high);
    return indirect_address_low | (indirect_address_high << 8U);
  }

  [[nodiscard]] auto get_immediate(decode::instruction const &instruction) const -> uint8_t {
    return std::get<uint8_t>(instruction.decoded_operand.value);
  }

  [[nodiscard]] auto get_relative_address(decode::instruction const &instruction) const -> uint16_t {
    const int8_t address_offset = std::get<uint8_t>(instruction.decoded_operand.value);
    return registers_.get(register_type::PC) + address_offset;
  }

private:
  static constexpr uint16_t stack_pointer_base = 0x100U;

  memory const &memory_;

  status_register status_register_{};

  registers registers_{};

  interrupts interrupts_{};

  uint64_t cycles_{};

  uint16_t idle_cycles_{};

  std::atomic<bool> is_running_{};

  cpu_step step_idle_ = step_idle();

  cpu_step step_execute_ = step_execute();

  cpu_step step_interrupt_ = step_interrupt();
};

cpu::cpu(memory const &memory) : impl_(new impl(memory)) {}

cpu::~cpu() = default;

auto cpu::initialize() -> void {
  impl_->initialize();
}

auto cpu::uninitialize() -> void {
  impl_->uninitialize();
}

auto cpu::step() -> uint8_t {
  return impl_->step();
}

auto cpu::reset() -> void {
  impl_->reset();
}

auto cpu::peek(uint16_t const address) const -> uint8_t {
  return impl_->peek(address);
}

auto cpu::read(uint16_t const address) const -> uint8_t {
  return impl_->read(address);
}

auto cpu::write(uint16_t const address, const uint8_t data) -> void {
  impl_->write(address, data);
}

auto cpu::get_state() const -> cpu_state {
  return impl_->get_state();
}

auto cpu::set_state(const cpu_state &state) -> void {
  return impl_->set_state(state);
}

auto cpu::interrupt(interrupt_type const interrupt, interrupt_state const state) -> void {
  return impl_->interrupt(interrupt, state);
}

auto cpu::idle(uint16_t const cycles) -> void {
  return impl_->idle(cycles);
}
