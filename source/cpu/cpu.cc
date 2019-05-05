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
#include "disassemble_internal.hh"
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
    const auto reset_routine = memory_.read_word(reset_vector) - 4;
    registers_.set(register_type::PC, reset_routine);

    // cycles for the cost of reset interrupt
    cycles_ += 7;
  }

  uint8_t read(const uint16_t address) const {
    boost::ignore_unused(address);
    return 0;
  }

  void write(const uint16_t address, const uint8_t data) {
    boost::ignore_unused(address);
    boost::ignore_unused(data);
  }

  cpu_state get_state() const {

    class listener : public disassemble::disassemble_listener {
    public:
      explicit listener(const registers &registers) : registers_(registers) {}

      ~listener() override = default;

      void on_decoded(decode::instruction &instruction) override {
        if (instruction.decoded_addressing_mode == addressing_mode_type::RELATIVE) {
          const auto address_relative = std::get<uint8_t>(instruction.decoded_operand.value);
          const auto address_absolute = registers_.get(register_type::PC) + address_relative + instruction.encoded_length_in_bytes;
          instruction.decoded_operand.value = static_cast<uint16_t>(address_absolute);
          instruction.decoded_addressing_mode = addressing_mode_type::ABSOLUTE;
        }
      }

      void on_disassembled(disassemble::instruction &instruction) override {
        boost::ignore_unused(instruction);
      }

    private:
      const registers &registers_;
    };

    auto fetched = fetch();
    auto disassembled = disassemble::disassemble(&fetched[0], fetched.size(), std::make_unique<listener>(registers_));

    cpu_state current_cpu_state;
    current_cpu_state.instruction = disassembled.instructions[0].opcode + disassembled.instructions[0].operand;
    current_cpu_state.instruction_bytes = fetched;
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

    push_pc();
    push_flags();

    const auto interrupt_vector = interrupts_.get_vector(type);
    registers_.set(register_type::PC, interrupt_vector);

    status_register_.set(status_flag::I);

    cycles_ += 7;
  }

  void push(const uint8_t value) {
    memory_.write(get_stack_pointer(), value);
    registers_.decrement(register_type::SP);
  }

  uint8_t pull() {
    registers_.increment(register_type::SP);
    return memory_.read(get_stack_pointer());
  }

  void push_flags() {
    const uint8_t flags = 0x10U | status_register_.get();
    push(flags);
  }

  void pull_flags() {
    const uint8_t flags = (pull() & 0xEFU) | 0x20U;
    status_register_.set(flags);
    registers_.set(register_type::SR, status_register_.get());
  }

  void push_pc() {
    const uint16_t pc = registers_.get(register_type::PC);
    push(pc >> 8U);
    push(pc);
  }

  void pull_pc() {
    const uint16_t low_byte = pull();
    const uint16_t high_byte = pull() << 8U;
    registers_.set(register_type::PC, (low_byte | high_byte));
  }

  uint16_t get_stack_pointer() const {
    return stack_pointer_base | registers_.get(register_type::SP);
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
    case opcode_type::BRK: {
      interrupt(interrupt_type::BRK);
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
    default: {
      break;
    }
    }
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
    const auto value = registers_.get(type);
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
    const auto value = registers_.get(type);
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
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address) + 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address) + 1;
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
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address) - 1;
      memory_.write(address, value);
      update_status_flag_zn(value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address) - 1;
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
    const auto ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      const auto value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      const auto address = get_absolute_y_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      bool is_page_crossed = false;
      const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register ^ value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
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
    const auto ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      const auto value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      const auto address = get_absolute_y_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      bool is_page_crossed = false;
      const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register | value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
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
      const auto value = registers_.get(register_type::AC);
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
      const auto value = pull();
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
    const auto register_y = registers_.get(register_type::Y);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      const auto value = get_immediate(instruction);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_y - value);
      update_status_flag_c(register_y >= value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
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
    const auto register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      const auto value = get_immediate(instruction);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_x - value);
      update_status_flag_c(register_x >= value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
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
    const auto register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      const auto value = get_immediate(instruction);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      const auto address = get_absolute_y_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      bool is_page_crossed = false;
      const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      update_status_flag_zn(register_ac - value);
      update_status_flag_c(register_ac >= value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
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
      const auto register_ac = static_cast<uint8_t>(registers_.get(register_type::AC));
      const auto value = register_ac >> 0x1U;
      registers_.set(register_type::AC, value);
      update_status_flag_c(register_ac & 0x1U);
      update_status_flag_zn(value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = value >> 0x1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      const auto shifted_value = value >> 0x1U;
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
    const auto has_carry = status_register_.is_set(status_flag::C);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      const auto value = static_cast<uint8_t>(registers_.get(register_type::AC));
      const auto shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x1U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value >> 0x1U) | (has_carry ? 1U << 7U : 0U);
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
    const auto has_carry = status_register_.is_set(status_flag::C);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ACCUMULATOR: {
      const auto value = static_cast<uint8_t>(registers_.get(register_type::AC));
      const auto shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
      memory_.write(address, shifted_value);
      update_status_flag_c(value & 0x80U);
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      auto is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      const auto shifted_value = static_cast<uint8_t>(value << 0x1U) | (has_carry ? 1U : 0U);
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
      const auto value = static_cast<uint8_t>(registers_.get(register_type::AC));
      const auto shifted_value = value << 1U;
      registers_.set(register_type::AC, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 5;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      const auto shifted_value = value << 1U;
      memory_.write(address, shifted_value);
      update_status_flag_c(std::bitset<8>(value).test(7));
      update_status_flag_zn(shifted_value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      const auto shifted_value = value << 1U;
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
    const auto ac_register = registers_.get(register_type::AC);
    const auto is_carry_set = status_register_.is_set(status_flag::C);
    uint16_t value = 0;
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      value = get_immediate(instruction);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      value = memory_.read(address);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      const auto address = get_absolute_y_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      bool is_page_crossed = false;
      const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ADC");
      break;
    }
    const auto subtraction = static_cast<int>(ac_register - value - (1 - (is_carry_set ? 1 : 0)));
    registers_.set(register_type::AC, static_cast<uint16_t>(subtraction));

    const auto caused_overflow = (static_cast<uint16_t>(ac_register ^ value) & 0x80U) != 0 &&
                                 (static_cast<uint16_t>(ac_register ^ registers_.get(register_type::AC)) & 0x80U) != 0;

    status_register_.set(status_flag::C, subtraction >= 0);
    status_register_.set(status_flag::V, caused_overflow);

    update_status_flag_zn(registers_.get(register_type::AC));
  }

  void execute_adc(const decode::instruction &instruction) {
    const auto ac_register = registers_.get(register_type::AC);
    const auto is_carry_set = status_register_.is_set(status_flag::C);
    uint16_t value = 0;
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      value = get_immediate(instruction);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      value = memory_.read(address);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      value = memory_.read(address);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      const auto address = get_absolute_y_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      bool is_page_crossed = false;
      const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
      value = memory_.read(address);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for ADC");
      break;
    }
    const auto addition = static_cast<uint16_t>(ac_register + value + (is_carry_set ? 1 : 0));
    registers_.set(register_type::AC, addition);

    const auto caused_overflow = (static_cast<uint16_t>(ac_register ^ value) & 0x80U) == 0 &&
                                 (static_cast<uint16_t>(ac_register ^ registers_.get(register_type::AC)) & 0x80U) != 0;

    status_register_.set(status_flag::C, addition > 0xFFU);
    status_register_.set(status_flag::V, caused_overflow);
    update_status_flag_zn(registers_.get(register_type::AC));
  }

  void execute_and(const decode::instruction &instruction) {
    const auto ac_register = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::IMMEDIATE: {
      const auto value = get_immediate(instruction);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 2;
      break;
    }
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      const auto address = get_absolute_y_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      bool is_page_crossed = false;
      const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
      registers_.set(register_type::AC, ac_register & value);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
      const auto value = memory_.read(address);
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
      const auto operand = get_zero_page_address(instruction);
      const auto address = memory_.read(operand);
      const auto address_bits = std::bitset<8>(address);
      address_bits.test(6) ? status_register_.set(status_flag::V) : status_register_.clear(status_flag::V);
      address_bits.test(7) ? status_register_.set(status_flag::N) : status_register_.clear(status_flag::N);
      (address & registers_.get(register_type::AC)) ? status_register_.clear(status_flag::Z) : status_register_.set(status_flag::Z);
      registers_.set(register_type::SR, status_register_.get());
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto operand = get_absolute_address(instruction);
      const auto address = memory_.read(operand);
      const auto address_bits = std::bitset<8>(address);
      address_bits.test(6) ? status_register_.set(status_flag::V) : status_register_.clear(status_flag::V);
      address_bits.test(7) ? status_register_.set(status_flag::N) : status_register_.clear(status_flag::N);
      (address & registers_.get(register_type::AC)) ? status_register_.clear(status_flag::Z) : status_register_.set(status_flag::Z);
      registers_.set(register_type::SR, status_register_.get());
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
        const uint8_t address_offset = std::get<uint8_t>(instruction.decoded_operand.value);
        const uint16_t previous_pc = registers_.get(register_type::PC);
        const uint16_t previous_page = previous_pc & 0xFF00U;
        const uint16_t next_pc = registers_.get(register_type::PC) + address_offset;
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
    boost::ignore_unused(instruction);
    // nothing to do for instruction.
    cycles_ += 2;
  }

  void execute_sty(const decode::instruction &instruction) {
    const auto register_y = registers_.get(register_type::Y);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      memory_.write(address, register_y);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      memory_.write(address, register_y);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
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
    const auto register_ac = registers_.get(register_type::AC);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_X: {
      const auto address = get_zero_page_x_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
      memory_.write(address, register_ac);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE_X: {
      bool is_page_crossed = false;
      const auto address = get_absolute_x_address(instruction, is_page_crossed);
      memory_.write(address, register_ac);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::ABSOLUTE_Y: {
      bool is_page_crossed = false;
      const auto address = get_absolute_y_address(instruction, is_page_crossed);
      memory_.write(address, register_ac);
      cycles_ += 4 + (is_page_crossed ? 1 : 0);
      break;
    }
    case addressing_mode_type::INDEXED_INDIRECT: {
      bool is_page_crossed = false;
      const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
      memory_.write(address, register_ac);
      cycles_ += 6;
      break;
    }
    case addressing_mode_type::INDIRECT_INDEXED: {
      bool is_page_crossed = false;
      const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
      memory_.write(address, register_ac);
      cycles_ += 5 + (is_page_crossed ? 1 : 0);
      break;
    }
    default:
      BOOST_STATIC_ASSERT("unexpected addressing mode for STA");
      break;
    }
  }

  void execute_stx(const decode::instruction &instruction) {
    const auto register_x = registers_.get(register_type::X);
    switch (instruction.decoded_addressing_mode) {
    case addressing_mode_type::ZERO_PAGE: {
      const auto address = get_zero_page_address(instruction);
      memory_.write(address, register_x);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::ZERO_PAGE_Y: {
      const auto address = get_zero_page_y_address(instruction);
      memory_.write(address, register_x);
      cycles_ += 4;
      break;
    }
    case addressing_mode_type::ABSOLUTE: {
      const auto address = get_absolute_address(instruction);
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
      const auto address = get_absolute_address(instruction);
      registers_.set(register_type::PC, address);
      cycles_ += 3;
      break;
    }
    case addressing_mode_type::INDIRECT: {
      const auto address = get_indirect_address(instruction);
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
      const auto value = get_immediate(instruction);
      registers_.set(register_type::AC, value);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        const auto address = get_zero_page_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_X: {
        const auto address = get_zero_page_x_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        const auto address = get_absolute_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_X: {
        bool is_page_crossed = false;
        const auto address = get_absolute_x_address(instruction, is_page_crossed);
        const auto value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        const auto address = get_absolute_y_address(instruction, is_page_crossed);
        const auto value = memory_.read(address);
        registers_.set(register_type::AC, value);
        cycles_ += 4 + (is_page_crossed ? 1 : 0);
        break;
      }
      case addressing_mode_type::INDEXED_INDIRECT: {
        bool is_page_crossed = false;
        const auto address = get_indexed_indirect_address(instruction, is_page_crossed);
        const auto memory = memory_.read(address);
        registers_.set(register_type::AC, memory);
        cycles_ += 6;
        break;
      }
      case addressing_mode_type::INDIRECT_INDEXED: {
        bool is_page_crossed = false;
        const auto address = get_indirect_indexed_address(instruction, is_page_crossed);
        const auto value = memory_.read(address);
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
      const auto value = get_immediate(instruction);
      registers_.set(register_type::Y, value);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        const auto address = get_zero_page_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::Y, value);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_X: {
        const auto address = get_zero_page_x_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::Y, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        const auto address = get_absolute_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::Y, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_X: {
        bool is_page_crossed = false;
        const auto address = get_absolute_x_address(instruction, is_page_crossed);
        const auto value = memory_.read(address);
        registers_.set(register_type::X, value);
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
      const auto value = get_immediate(instruction);
      registers_.set(register_type::X, value);
      cycles_ += 2;
      break;
    }
    case operand_type::MEMORY:
      switch (instruction.decoded_addressing_mode) {
      case addressing_mode_type::ZERO_PAGE: {
        const auto address = get_zero_page_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::X, value);
        cycles_ += 3;
        break;
      }
      case addressing_mode_type::ZERO_PAGE_Y: {
        const auto address = get_zero_page_y_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::X, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE: {
        const auto address = get_absolute_address(instruction);
        const auto value = memory_.read(address);
        registers_.set(register_type::X, value);
        cycles_ += 4;
        break;
      }
      case addressing_mode_type::ABSOLUTE_Y: {
        bool is_page_crossed = false;
        const auto address = get_absolute_y_address(instruction, is_page_crossed);
        const auto value = memory_.read(address);
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
    const auto bits = std::bitset<8>(binary);
    status_register_.set(status_flag::N, bits.test(7));
    registers_.set(register_type::SR, status_register_.get());
  }

  void update_status_flag_z(const uint8_t binary) {
    const auto bits = std::bitset<8>(binary);
    status_register_.set(status_flag::Z, !bits.any());
    registers_.set(register_type::SR, status_register_.get());
  }

  void update_status_flag_c(const bool has_carry) {
    status_register_.set(status_flag::C, has_carry);
    registers_.set(register_type::SR, status_register_.get());
  }

  uint16_t get_zero_page_address(const decode::instruction &instruction) const {
    return std::get<uint8_t>(instruction.decoded_operand.value) & 0xFFU;
  }

  uint16_t get_zero_page_x_address(const decode::instruction &instruction) const {
    const auto address = std::get<uint8_t>(instruction.decoded_operand.value);
    const auto x_register = registers_.get(register_type::X);
    return static_cast<uint16_t>(address + x_register) & 0xFFU;
  }

  uint16_t get_zero_page_y_address(const decode::instruction &instruction) const {
    const auto address = std::get<uint8_t>(instruction.decoded_operand.value);
    const auto y_register = registers_.get(register_type::Y);
    return static_cast<uint16_t>(address + y_register) & 0xFFU;
  }

  uint16_t get_absolute_address(const decode::instruction &instruction) const {
    return std::get<uint16_t>(instruction.decoded_operand.value);
  }

  uint16_t get_absolute_y_address(const decode::instruction &instruction, bool &is_page_crossed) const {
    const auto address = std::get<uint16_t>(instruction.decoded_operand.value);
    const auto y_register = registers_.get(register_type::Y);
    is_page_crossed = (static_cast<uint16_t>(address + y_register) & 0xFFU) < y_register;
    return address + y_register;
  }

  uint16_t get_absolute_x_address(const decode::instruction &instruction, bool &is_page_crossed) const {
    const auto address = std::get<uint16_t>(instruction.decoded_operand.value);
    const auto x_register = registers_.get(register_type::X);
    is_page_crossed = (static_cast<uint16_t>(address + x_register) & 0xFFU) < x_register;
    return address + x_register;
  }

  uint16_t get_indexed_indirect_address(const decode::instruction &instruction, bool &is_page_crossed) const {
    const auto operand = std::get<uint8_t>(instruction.decoded_operand.value);
    const auto x_register = registers_.get(register_type::X);
    const auto address = static_cast<uint16_t>(operand + x_register) & 0xFFU;
    is_page_crossed = address < x_register;
    return get_indirect_address(address);
  }

  uint16_t get_indirect_indexed_address(const decode::instruction &instruction, bool &is_page_crossed) const {
    const auto operand = std::get<uint8_t>(instruction.decoded_operand.value);
    const auto address = memory_.read(operand & 0xFFU);
    const auto y_register = registers_.get(register_type::Y);
    is_page_crossed = (static_cast<uint16_t>(address + y_register) & 0xFFU) < y_register;
    return address + y_register;
  }

  uint16_t get_indirect_address(const decode::instruction &instruction) const {
    return get_indirect_address(std::get<uint16_t>(instruction.decoded_operand.value));
  }

  uint16_t get_indirect_address(const uint16_t address) const {
    const auto address_page = address & (0xFF00U);
    const auto address_high = (address + 1U) & 0xFFU;
    const auto indirect_address_low = memory_.read(address);
    const auto indirect_address_high = memory_.read(address_page | address_high);
    return indirect_address_low | (indirect_address_high << 8U);
  }

  uint8_t get_immediate(const decode::instruction &instruction) const {
    return std::get<uint8_t>(instruction.decoded_operand.value);
  }

private:
  static constexpr uint16_t stack_pointer_base = 0x100U;

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
