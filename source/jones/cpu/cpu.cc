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
#include <atomic>
#include <boost/coroutine2/all.hpp>
#include <boost/format.hpp>

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

using coroutine = boost::coroutines2::coroutine<uint64_t>;

namespace {

inline auto co_await(coroutine::pull_type &coroutine) -> auto {
  return coroutine()().get();
}

} // namespace

class cpu::impl final {
public:
  explicit impl(memory const &memory) : memory_(memory) {}

  auto initialize() -> void {
    initialize_coroutines();
    reset();
  }

  auto uninitialize() -> void {
    uninitialize_coroutines();
  }

  auto step() -> uint8_t {
    auto const initial_cycles = cycles_;
    if (idle_cycles_ > 0) {
      cycles_ += co_await(*step_idle_);
    } else if (is_interrupt_pending()) {
      cycles_ += co_await(*step_interrupt_);
    } else {
      cycles_ += co_await(*step_execute_);
    }
    return cycles_ - initial_cycles;
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
          const uint16_t address_absolute = registers_.get(register_type::PC) + address_relative + instruction.encoded_length_in_bytes;
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

  auto step_idle(coroutine::push_type &co_yield) -> void {
    co_yield(0);
    while (is_running_) {
      idle_cycles_ -= 1;
      co_yield(1);
    }
  }

  auto step_execute(coroutine::push_type &co_yield) -> void {
    co_yield(0);
    while (is_running_) {
      auto const fetched = fetch();
      auto const decoded = decode(fetched);
      if (decoded.decoded_result == decode::result::SUCCESS) {
        registers_.increment_by(register_type::PC, decoded.encoded_length_in_bytes);
        execute(decoded);
      } else {
        BOOST_STATIC_ASSERT("unable to step cpu; decoded invalid instruction");
        interrupt(interrupt_type::BRK, interrupt_state::SET);
      }
      co_yield(0);
    }
  }

  auto step_interrupt(coroutine::push_type &co_yield) -> void {
    co_yield(0);
    while (is_running_) {
      auto const triggered_interrupt = interrupts_.get_triggered();
      if (triggered_interrupt == interrupt_type::NONE) {
        return;
      }
      push_pc();
      push_flags();
      auto const interrupt_vector = interrupts_.get_vector(triggered_interrupt);
      auto const interrupt_routine = memory_.read_word(interrupt_vector);
      registers_.set(register_type::PC, interrupt_routine);
      status_register_.set(status_flag::I);
      registers_.set(register_type::SR, status_register_.get());
      interrupts_.set_state(triggered_interrupt, false);
      co_yield(7);
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
    const uint16_t pc = registers_.get(register_type::PC);
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

  auto execute(decode::instruction const &instruction) -> void {
    switch (instruction.decoded_opcode.type) {
    case opcode_type::BRK: {
      interrupt(interrupt_type::BRK, interrupt_state::SET);
      break;
    }
    case opcode_type::PHP: {
      execute_php(instruction);
      break;
    }
    case opcode_type::PLP: {
      execute_plp(instruction);
      break;
    }
    case opcode_type::PHA: {
      execute_pha(instruction);
      break;
    }
    case opcode_type::PLA: {
      execute_pla(instruction);
      break;
    }
    case opcode_type::JMP: {
      execute_jmp(instruction);
      break;
    }
    case opcode_type::LDA: {
      execute_lda(instruction);
      break;
    }
    case opcode_type::LDX: {
      execute_ldx(instruction);
      break;
    }
    case opcode_type::LDY: {
      execute_ldy(instruction);
      break;
    }
    case opcode_type::STA: {
      execute_sta(instruction);
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
    case opcode_type::CLC: {
      execute_clc(instruction);
      break;
    }
    case opcode_type::SEC: {
      execute_sec(instruction);
      break;
    }
    case opcode_type::CLI: {
      execute_cli(instruction);
      break;
    }
    case opcode_type::SEI: {
      execute_sei(instruction);
      break;
    }
    case opcode_type::CLD: {
      execute_cld(instruction);
      break;
    }
    case opcode_type::SED: {
      execute_sed(instruction);
      break;
    }
    case opcode_type::CLV: {
      execute_clv(instruction);
      break;
    }
    case opcode_type::BCS: {
      execute_bcs(instruction);
      break;
    }
    case opcode_type::BCC: {
      execute_bcc(instruction);
      break;
    }
    case opcode_type::BVS: {
      execute_bvs(instruction);
      break;
    }
    case opcode_type::BVC: {
      execute_bvc(instruction);
      break;
    }
    case opcode_type::BEQ: {
      execute_beq(instruction);
      break;
    }
    case opcode_type::BNE: {
      execute_bne(instruction);
      break;
    }
    case opcode_type::BMI: {
      execute_bmi(instruction);
      break;
    }
    case opcode_type::BPL: {
      execute_bpl(instruction);
      break;
    }
    case opcode_type::BIT: {
      execute_bit(instruction);
      break;
    }
    case opcode_type::ADC: {
      execute_adc(instruction);
      break;
    }
    case opcode_type::SBC: {
      execute_sbc(instruction);
      break;
    }
    case opcode_type::AND: {
      execute_and(instruction);
      break;
    }
    case opcode_type::ASL: {
      execute_asl(instruction);
      break;
    }
    case opcode_type::LSR: {
      execute_lsr(instruction);
      break;
    }
    case opcode_type::ROR: {
      execute_ror(instruction);
      break;
    }
    case opcode_type::ROL: {
      execute_rol(instruction);
      break;
    }
    case opcode_type::CMP: {
      execute_cmp(instruction);
      break;
    }
    case opcode_type::CPX: {
      execute_cpx(instruction);
      break;
    }
    case opcode_type::CPY: {
      execute_cpy(instruction);
      break;
    }
    case opcode_type::RTS: {
      execute_rts(instruction);
      break;
    }
    case opcode_type::ORA: {
      execute_ora(instruction);
      break;
    }
    case opcode_type::EOR: {
      execute_eor(instruction);
      break;
    }
    case opcode_type::INY: {
      execute_iny(instruction);
      break;
    }
    case opcode_type::INX: {
      execute_inx(instruction);
      break;
    }
    case opcode_type::INC: {
      execute_inc(instruction);
      break;
    }
    case opcode_type::DEC: {
      execute_dec(instruction);
      break;
    }
    case opcode_type::DEY: {
      execute_dey(instruction);
      break;
    }
    case opcode_type::DEX: {
      execute_dex(instruction);
      break;
    }
    case opcode_type::TAY: {
      execute_tay(instruction);
      break;
    }
    case opcode_type::TYA: {
      execute_tya(instruction);
      break;
    }
    case opcode_type::TAX: {
      execute_tax(instruction);
      break;
    }
    case opcode_type::TXA: {
      execute_txa(instruction);
      break;
    }
    case opcode_type::TSX: {
      execute_tsx(instruction);
      break;
    }
    case opcode_type::TXS: {
      execute_txs(instruction);
      break;
    }
    case opcode_type::RTI: {
      execute_rti(instruction);
      break;
    }
    case opcode_type::LAX: {
      execute_lax(instruction);
      break;
    }
    case opcode_type::SAX: {
      execute_sax(instruction);
      break;
    }
    case opcode_type::DCP: {
      execute_dcp(instruction);
      break;
    }
    case opcode_type::ISC: {
      execute_isc(instruction);
      break;
    }
    case opcode_type::SLO: {
      execute_slo(instruction);
      break;
    }
    case opcode_type::RLA: {
      execute_rla(instruction);
      break;
    }
    case opcode_type::SRE: {
      execute_sre(instruction);
      break;
    }
    case opcode_type::RRA: {
      execute_rra(instruction);
      break;
    }
    default: {
      break;
    }
    }
  }

  auto execute_rra(decode::instruction const &instruction) -> void {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      cycles_ += 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rra(value));
      cycles_ += 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute RRA");
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

  auto execute_sre(decode::instruction const &instruction) -> void {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      cycles_ += 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_sre(value));
      cycles_ += 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute SRE");
      break;
    }
    }
  }

  uint8_t execute_sre(const uint8_t value) {
    auto const shifted_value = value >> 0x1U;
    auto const ac_register = registers_.get(register_type::AC);
    auto const new_ac_register = ac_register ^ shifted_value;
    registers_.set(register_type::AC, new_ac_register);
    update_status_flag_c(value & 0x1U);
    update_status_flag_zn(new_ac_register);
    return shifted_value;
  }

  void execute_rla(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      cycles_ += 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address);
      memory_.write(address, execute_rla(value));
      cycles_ += 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute RLA");
      break;
    }
    }
  }

  uint8_t execute_rla(const uint8_t value) {
    auto const has_carry = status_register_.is_set(status_flag::C);
    auto const shifted_value = (value << 1) | (has_carry ? 1 : 0);
    auto const ac_register = registers_.get(register_type::AC);
    auto const new_ac_register = ac_register & shifted_value;
    registers_.set(register_type::AC, new_ac_register & 0xFF);
    update_status_flag_c((value >> 7) & 0x1U);
    update_status_flag_zn(new_ac_register);
    return shifted_value;
  }

  void execute_slo(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));

      auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
      registers_.set(register_type::AC, ora_value);
      update_status_flag_zn(ora_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));

      auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
      registers_.set(register_type::AC, ora_value);
      update_status_flag_zn(ora_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));

      auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
      registers_.set(register_type::AC, ora_value);
      update_status_flag_zn(ora_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));

      auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
      registers_.set(register_type::AC, ora_value);
      update_status_flag_zn(ora_value);
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));

      auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
      registers_.set(register_type::AC, ora_value);
      update_status_flag_zn(ora_value);
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));

      auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
      registers_.set(register_type::AC, ora_value);
      update_status_flag_zn(ora_value);
      cycles_ += 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));

      auto const ora_value = static_cast<uint8_t>((registers_.get(register_type::AC) | shifted_value) & 0xFF);
      registers_.set(register_type::AC, ora_value);
      update_status_flag_zn(ora_value);
      cycles_ += 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute SLO");
      break;
    }
    }
  }

  void execute_isc(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      cycles_ += 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      execute_sbc(value);
      cycles_ += 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute ISC");
      break;
    }
    }
  }

  void execute_dcp(const decode::instruction &instruction) {
    auto const register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      cycles_ += 7;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      cycles_ += 8;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_c(register_ac - value >= 0);
      update_status_flag_zn(register_ac - value);
      cycles_ += 8;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for DCP");
      break;
    }
    }
  }

  void execute_sax(const decode::instruction &instruction) {
    auto const register_ac = registers_.get(register_type::AC);
    auto const register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_ac & register_x);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_Y: {
      auto const address = get_zero_page_y_address(instruction);
      memory_.write(address, register_ac & register_x);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_ac & register_x);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      memory_.write(address, register_ac & register_x);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      memory_.write(address, register_ac & register_x);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      memory_.write(address, register_ac & register_x);
      cycles_ += 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for SAX");
      break;
    }
  }

  void execute_lax(const decode::instruction &instruction) {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, value);
      registers_.set(register_type::X, value);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_Y: {
        auto const address = get_zero_page_y_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        auto const address = get_absolute_y_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        cycles_ += 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::INDEXED_INDIRECT: {
        auto const address = get_indexed_indirect_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        cycles_ += 6;
        break;
      }
      case addressing_mode_type::INDIRECT_INDEXED: {
        bool is_page_crossed = false;
        auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        registers_.set(register_type::X, value);
        cycles_ += 5 + (is_page_crossed ? 1 : 0);
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LAX");
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LAX");
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  void execute_rti(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      pull_flags();
      pull_pc();
      cycles_ += 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for RTI");
      break;
    }
  }

  void execute_transfer(const decode::instruction &instruction, const register_type source, const register_type destination, const bool should_update_status = true) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      registers_.set(destination, registers_.get(source));
      cycles_ += 2;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute transfer");
      break;
    }
    if (should_update_status) {
      update_status_flag_zn(registers_.get(destination));
    }
  }

  void execute_tay(const decode::instruction &instruction) {
    execute_transfer(instruction, register_type::AC, register_type::Y);
  }

  void execute_tya(const decode::instruction &instruction) {
    execute_transfer(instruction, register_type::Y, register_type::AC);
  }

  void execute_tax(const decode::instruction &instruction) {
    execute_transfer(instruction, register_type::AC, register_type::X);
  }

  void execute_txa(const decode::instruction &instruction) {
    execute_transfer(instruction, register_type::X, register_type::AC);
  }

  void execute_tsx(const decode::instruction &instruction) {
    execute_transfer(instruction, register_type::SP, register_type::X);
  }

  void execute_txs(const decode::instruction &instruction) {
    execute_transfer(instruction, register_type::X, register_type::SP, false);
  }

  void execute_decrement(const decode::instruction &instruction, const register_type type) {
    auto const value = registers_.get(type);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      registers_.set(type, value - 1);
      cycles_ += 2;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute decrement");
      break;
    }
    update_status_flag_zn(registers_.get(type));
  }

  void execute_dey(const decode::instruction &instruction) {
    execute_decrement(instruction, register_type::Y);
  }

  void execute_dex(const decode::instruction &instruction) {
    execute_decrement(instruction, register_type::X);
  }

  void execute_increment(const decode::instruction &instruction, const register_type type) {
    auto const value = registers_.get(type);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      registers_.set(type, value + 1);
      cycles_ += 2;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for execute increment");
      break;
    }
    update_status_flag_zn(registers_.get(type));
  }

  void execute_iny(const decode::instruction &instruction) {
    execute_increment(instruction, register_type::Y);
  }

  void execute_inx(const decode::instruction &instruction) {
    execute_increment(instruction, register_type::X);
  }

  void execute_inc(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for INC");
      break;
    }
    }
  }

  void execute_dec(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for DEC");
      break;
    }
    }
  }

  void execute_eor(const decode::instruction &instruction) {
    auto const ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for EOR");
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  void execute_ora(const decode::instruction &instruction) {
    auto const ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ORA");
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  void execute_pha(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      auto const value = registers_.get(register_type::AC);
      push(value);
      cycles_ += 3;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PHA");
      break;
    }
  }

  void execute_pla(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      auto const value = pull();
      registers_.set(register_type::AC, value);
      update_status_flag_zn(value);
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PLA");
      break;
    }
  }

  void execute_php(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      push_flags();
      cycles_ += 3;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PHP");
      break;
    }
  }

  void execute_plp(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      pull_flags();
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for PLP");
      break;
    }
  }

  void execute_cpy(const decode::instruction &instruction) {
    auto const register_y = registers_.get(register_type::Y);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for CPY");
      break;
    }
  }

  void execute_cpx(const decode::instruction &instruction) {
    auto const register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for CPX");
      break;
    }
  }

  void execute_cmp(const decode::instruction &instruction) {
    auto const register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for CMP");
      break;
    }
  }

  void execute_lsr(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const register_ac = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const value = register_ac >> 0x1U;
      registers_.set(register_type::AC, value);
      update_status_flag_c(register_ac & 0x1U);
      update_status_flag_zn(value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for LSR");
      break;
    }
    }
  }

  void execute_ror(const decode::instruction &instruction) {
    auto const has_carry = status_register_.is_set(status_flag::C);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const value = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for ROR");
      break;
    }
    }
  }

  void execute_rol(const decode::instruction &instruction) {
    auto const has_carry = status_register_.is_set(status_flag::C);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const value = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 7;
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("unexpected addressing mode for ROL");
      break;
    }
    }
  }

  void execute_asl(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      auto const value = static_cast<uint8_t>(registers_.get(register_type::AC));
      auto const shifted_value = value << 1U;
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      auto const value = memory_.read(address);
      auto const shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 7;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ASL");
      break;
    }
  }

  void execute_sbc(const decode::instruction &instruction) {
    uint16_t value = 0;
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      value = get_immediate(instruction);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      value = memory_.read(address);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      value = memory_.read(address);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ADC");
      break;
    }
    execute_sbc(value);
  }

  void execute_sbc(const uint8_t value) {
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

  void execute_adc(const decode::instruction &instruction) {
    auto const ac_register = registers_.get(register_type::AC);
    auto const is_carry_set = status_register_.is_set(status_flag::C);
    uint16_t value = 0;
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      value = get_immediate(instruction);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      value = memory_.read(address);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      value = memory_.read(address);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ADC");
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

  void execute_and(const decode::instruction &instruction) {
    auto const ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      auto const address = get_absolute_x_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      auto const address = get_absolute_y_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
      auto const value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for AND");
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  void execute_bit(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      auto const value = memory_.read(address);
      auto const bits = std::bitset<8>(value);
      bits.test(6) ? status_register_.set(status_flag::V) : status_register_.clear(status_flag::V);
      bits.test(7) ? status_register_.set(status_flag::N) : status_register_.clear(status_flag::N);
      update_status_flag_z(value & registers_.get(register_type::AC));
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      auto const value = memory_.read(address);
      auto const bits = std::bitset<8>(value);
      bits.test(6) ? status_register_.set(status_flag::V) : status_register_.clear(status_flag::V);
      bits.test(7) ? status_register_.set(status_flag::N) : status_register_.clear(status_flag::N);
      update_status_flag_z(value & registers_.get(register_type::AC));
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for BIT");
      break;
    }
  }

  void execute_bmi(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::N, true);
  }

  void execute_bpl(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::N, false);
  }

  void execute_bne(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::Z, false);
  }

  void execute_beq(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::Z, true);
  }

  void execute_bvc(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::V, false);
  }

  void execute_bvs(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::V, true);
  }

  void execute_bcc(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::C, false);
  }

  void execute_bcs(const decode::instruction &instruction) {
    execute_branch(instruction, status_flag::C, true);
  }

  void execute_branch(const decode::instruction &instruction, const status_flag flag, const bool is_flag_set) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::RELATIVE: {
      cycles_ += 2;
      if (status_register_.is_set(flag) == is_flag_set) {
        cycles_ += 1;
        const uint16_t previous_pc = registers_.get(register_type::PC);
        const uint16_t previous_page = previous_pc & 0xFF00U;
        const uint16_t next_pc = get_relative_address(instruction);
        const uint16_t next_page = next_pc & 0xFF00U;
        if (previous_page != next_page) {
          cycles_ += 1;
        }
        registers_.set(register_type::PC, next_pc);
      }
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for BRANCH");
      break;
    }
  }

  void execute_clv(const decode::instruction &instruction) {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::V);
    registers_.set(register_type::SR, status_register_.get());
    cycles_ += 2;
  }

  void execute_cld(const decode::instruction &instruction) {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::D);
    registers_.set(register_type::SR, status_register_.get());
    cycles_ += 2;
  }

  void execute_sed(const decode::instruction &instruction) {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.set(status_flag::D);
    registers_.set(register_type::SR, status_register_.get());
    cycles_ += 2;
  }

  void execute_cli(const decode::instruction &instruction) {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::I);
    registers_.set(register_type::SR, status_register_.get());
    cycles_ += 2;
  }

  void execute_sei(const decode::instruction &instruction) {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.set(status_flag::I);
    registers_.set(register_type::SR, status_register_.get());
    cycles_ += 2;
  }

  void execute_clc(const decode::instruction &instruction) {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.clear(status_flag::C);
    registers_.set(register_type::SR, status_register_.get());
    cycles_ += 2;
  }

  void execute_sec(const decode::instruction &instruction) {
    BOOST_ASSERT(instruction.decoded_operand.type == operand_type::NONE);
    status_register_.set(status_flag::C);
    registers_.set(register_type::SR, status_register_.get());
    cycles_ += 2;
  }

  void execute_jsr(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ABSOLUTE: {
      registers_.decrement(register_type::PC);
      push_pc();
      const uint16_t address = get_absolute_address(instruction);
      registers_.set(register_type::PC, address);
      cycles_ += 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for JSR");
      break;
    }
  }

  void execute_rts(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT: {
      pull_pc();
      registers_.increment(register_type::PC);
      cycles_ += 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for RTS");
      break;
    }
  }

  void execute_nop(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMPLICIT:
      cycles_ += 2;
      break;
    case addressing_mode_type::IMMEDIATE:
      cycles_ += 2;
      break;
    case addressing_mode_type::ZERO_PAGE:
      cycles_ += 3;
      break;
    case addressing_mode_type::ZERO_PAGE_X:
      cycles_ += 4;
      break;
    case addressing_mode_type::ABSOLUTE:
      cycles_ += 4;
      break;
    case addressing_mode_type::ABSOLUTE_X: {
      auto is_page_crossed = false;
      (void)get_absolute_x_address(instruction, is_page_crossed);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for NOP");
      break;
    }
  }

  void execute_sty(const decode::instruction &instruction) {
    auto const register_y = registers_.get(register_type::Y);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_y);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      memory_.write(address, register_y);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_y);
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STY");
      break;
    }
  }

  void execute_sta(const decode::instruction &instruction) {
    auto const register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      auto const address = get_zero_page_x_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto const address = get_absolute_x_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      auto const address = get_absolute_y_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      auto const address = get_indexed_indirect_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      auto const address = get_indirect_indexed_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 6;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STA");
      break;
    }
  }

  void execute_stx(const decode::instruction &instruction) {
    auto const register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      auto const address = get_zero_page_address(instruction);
      memory_.write(address, register_x);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_Y: {
      auto const address = get_zero_page_y_address(instruction);
      memory_.write(address, register_x);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      memory_.write(address, register_x);
      cycles_ += 4;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STX");
      break;
    }
  }

  void execute_jmp(const decode::instruction &instruction) {
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ABSOLUTE: {
      auto const address = get_absolute_address(instruction);
      registers_.set(register_type::PC, address);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::INDIRECT: {
      auto const address = get_indirect_address(instruction);
      registers_.set(register_type::PC, address);
      cycles_ += 5;
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for JMP");
      break;
    }
  }

  void execute_lda(const decode::instruction &instruction) {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::AC, value);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_X: {
        auto const address = get_zero_page_x_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_X: {
        bool is_page_crossed = false;
        auto const address = get_absolute_x_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        auto const address = get_absolute_y_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::INDEXED_INDIRECT: {
        auto const address = get_indexed_indirect_address(instruction);
        auto const memory = memory_.read(address);
        registers_.set(register_type::AC, memory);
        cycles_ += 6;
        break;
      }
      case addressing_mode_type::INDIRECT_INDEXED: {
        bool is_page_crossed = false;
        auto const address = get_indirect_indexed_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 5 + (is_page_crossed ? 1 : 0);
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LDA");
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LDA");
      break;
    }
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  void execute_ldy(const decode::instruction &instruction) {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::Y, value);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_X: {
        auto const address = get_zero_page_x_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_X: {
        bool is_page_crossed = false;
        auto const address = get_absolute_x_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::Y, value);
        cycles_ += 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      default: {
        BOOST_STATIC_ASSERT("unexpected addressing mode for LDY");
        break;
      }
      }
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected operand type for LDY");
      break;
    }
    update_status_flag_zn(registers_.get(register_type::Y));
  }

  void execute_ldx(const decode::instruction &instruction) {
    switch (instruction.decoded_operand.type) {
    case operand_type::IMMEDIATE: {
      auto const value = get_immediate(instruction);
      registers_.set(register_type::X, value);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        auto const address = get_zero_page_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_Y: {
        auto const address = get_zero_page_y_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        auto const address = get_absolute_address(instruction);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        auto const address = get_absolute_y_address(instruction, is_page_crossed);
        auto const value = memory_.read(address);
        registers_.set(register_type::X, value);
        cycles_ += 4 + (is_page_crossed ? 1 : 0);
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

  [[nodiscard]] auto get_zero_page_address(const decode::instruction &instruction) const -> uint16_t {
    return std::get<uint8_t>(instruction.decoded_operand.value) & 0xFFU;
  }

  [[nodiscard]] auto get_zero_page_x_address(const decode::instruction &instruction) const -> uint16_t {
    auto const address = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const x_register = registers_.get(register_type::X);
    return static_cast<uint16_t>(address + x_register) & 0xFFU;
  }

  [[nodiscard]] auto get_zero_page_y_address(const decode::instruction &instruction) const -> uint16_t {
    auto const address = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const y_register = registers_.get(register_type::Y);
    return static_cast<uint16_t>(address + y_register) & 0xFFU;
  }

  [[nodiscard]] auto get_absolute_address(const decode::instruction &instruction) const -> uint16_t {
    return std::get<uint16_t>(instruction.decoded_operand.value);
  }

  [[nodiscard]] auto get_absolute_y_address(const decode::instruction &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_absolute_y_address(instruction, is_page_crossed);
  }

  auto get_absolute_y_address(const decode::instruction &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const address = std::get<uint16_t>(instruction.decoded_operand.value);
    auto const y_register = registers_.get(register_type::Y);
    is_page_crossed = (static_cast<uint16_t>(address + y_register) & 0xFFU) < y_register;
    return address + y_register;
  }

  [[nodiscard]] auto get_absolute_x_address(const decode::instruction &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_absolute_x_address(instruction, is_page_crossed);
  }

  auto get_absolute_x_address(const decode::instruction &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const address = std::get<uint16_t>(instruction.decoded_operand.value);
    auto const x_register = registers_.get(register_type::X);
    is_page_crossed = (static_cast<uint16_t>(address + x_register) & 0xFFU) < x_register;
    return address + x_register;
  }

  [[nodiscard]] auto get_indexed_indirect_address(const decode::instruction &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_indexed_indirect_address(instruction, is_page_crossed);
  }

  auto get_indexed_indirect_address(const decode::instruction &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const operand = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const x_register = registers_.get(register_type::X);
    auto const address = static_cast<uint16_t>(operand + x_register) & 0xFFU;
    is_page_crossed = address < x_register;
    return get_indirect_address(address);
  }

  [[nodiscard]] auto get_indirect_indexed_address(const decode::instruction &instruction) const -> uint16_t {
    auto is_page_crossed = false;
    return get_indirect_indexed_address(instruction, is_page_crossed);
  }

  auto get_indirect_indexed_address(const decode::instruction &instruction, bool &is_page_crossed) const -> uint16_t {
    auto const operand = std::get<uint8_t>(instruction.decoded_operand.value);
    auto const address = get_indirect_address(operand);
    auto const y_register = registers_.get(register_type::Y);
    is_page_crossed = (static_cast<uint16_t>(address + y_register) & 0xFFU) < y_register;
    return address + y_register;
  }

  [[nodiscard]] auto get_indirect_address(const decode::instruction &instruction) const -> uint16_t {
    return get_indirect_address(std::get<uint16_t>(instruction.decoded_operand.value));
  }

  [[nodiscard]] auto get_indirect_address(const uint16_t address) const -> uint16_t {
    auto const address_page = address & 0xFF00U;
    auto const address_high = (address + 1U) & 0xFFU;
    auto const indirect_address_low = memory_.read(address);
    auto const indirect_address_high = memory_.read(address_page | address_high);
    return indirect_address_low | (indirect_address_high << 8U);
  }

  [[nodiscard]] auto get_immediate(const decode::instruction &instruction) const -> uint8_t {
    return std::get<uint8_t>(instruction.decoded_operand.value);
  }

  [[nodiscard]] auto get_relative_address(const decode::instruction &instruction) const -> uint16_t {
    const int8_t address_offset = std::get<uint8_t>(instruction.decoded_operand.value);
    return registers_.get(register_type::PC) + address_offset;
  }

  auto initialize_coroutines() -> void {
    is_running_ = true;
    step_idle_ = std::make_unique<coroutine::pull_type>(std::bind(&impl::step_idle, this, std::placeholders::_1));
    step_execute_ = std::make_unique<coroutine::pull_type>(std::bind(&impl::step_execute, this, std::placeholders::_1));
    step_interrupt_ = std::make_unique<coroutine::pull_type>(std::bind(&impl::step_interrupt, this, std::placeholders::_1));
  }

  auto uninitialize_coroutines() -> void {
    is_running_ = false;
    step_idle_.reset();
    step_execute_.reset();
    step_interrupt_.reset();
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

  std::unique_ptr<coroutine::pull_type> step_idle_;

  std::unique_ptr<coroutine::pull_type> step_execute_;

  std::unique_ptr<coroutine::pull_type> step_interrupt_;
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

auto cpu::peek(const uint16_t address) const -> uint8_t {
  return impl_->peek(address);
}

auto cpu::read(const uint16_t address) const -> uint8_t {
  return impl_->read(address);
}

auto cpu::write(const uint16_t address, const uint8_t data) -> void {
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

auto cpu::idle(const uint16_t cycles) -> void {
  return impl_->idle(cycles);
}
