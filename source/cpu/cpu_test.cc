//
// MIT License
//
// Copyright 2018-2019
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
#define BOOST_TEST_MODULE cpu_test

#include <boost/test/unit_test.hpp>
#include <variant>

#include "decode.hh"
#include "disassemble.hh"

namespace jde = jones::decode;
namespace jdi = jones::disassemble;

namespace {

void check_instruction_invalid(const jde::instruction &instruction) {
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::INVALID);
  BOOST_CHECK(instruction.decoded_opcode.value == 0);

  BOOST_CHECK(instruction.decoded_operand.type == operand_type::INVALID);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0x00));
}

void check_instruction_no_operand(const jde::instruction &instruction) {
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::NONE);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0x00));
}

} // namespace

BOOST_AUTO_TEST_CASE(cpu_test) { BOOST_CHECK(true); }

//
// Test: Disassembles AND instruction with opcode 0x29.
//
BOOST_AUTO_TEST_CASE(disasemble_0x29_and_instruction_valid) {

  uint8_t and_instruction[] = {0x29, 0x2C};

  const auto instructions = jdi::disassemble(and_instruction, sizeof(and_instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "AND #$2C");
}

//
// Test: Decodes an instruction that is too small.  In this case, an empty instruction.
//
BOOST_AUTO_TEST_CASE(decode_invalid_instruction_too_small) {

  uint8_t empty_instruction[] = {};

  const auto instruction = jde::decode(empty_instruction, sizeof(empty_instruction));
  check_instruction_invalid(instruction);
}

//
// Test: Decodes opcode value 0x00.
//
BOOST_AUTO_TEST_CASE(decode_0x00_brk_instruction_valid) {

  uint8_t brk_instruction[] = {0x00};

  const auto instruction = jde::decode(brk_instruction, sizeof(brk_instruction));

  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::BRK);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x00);

  check_instruction_no_operand(instruction);
}

//
// Test: Disassembles opcode value 0x00.
//
BOOST_AUTO_TEST_CASE(disasemble_0x00_brk_instruction_valid) {

  uint8_t brk_instruction[] = {0x00};

  const auto instructions = jdi::disassemble(brk_instruction, sizeof(brk_instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "BRK");
}

//
// Test: Decodes opcode value 0x01 with operand value set to 0xFF
//
BOOST_AUTO_TEST_CASE(decode_0x01_ora_instruction_valid) {

  uint8_t ora_instruction[] = {0x01, 0xFF};

  const auto instruction = jde::decode(ora_instruction, sizeof(ora_instruction));

  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ORA);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x01);

  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x01 with missing operand value.
//
BOOST_AUTO_TEST_CASE(decode_0x01_ora_instruction_invalid_too_small) {

  uint8_t ora_instruction[] = {0x01};

  const auto instruction = jde::decode(ora_instruction, sizeof(ora_instruction));

  check_instruction_invalid(instruction);
}

//
// Test: Disassembles opcode value 0x01.
//
BOOST_AUTO_TEST_CASE(disasemble_0x01_ora_instruction_valid) {

  uint8_t ora_instruction[] = {0x01, 0xFF};

  const auto instructions = jdi::disassemble(ora_instruction, sizeof(ora_instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "ORA ($FF,X)");
}

//
// Test: Decodes opcode value 0x02.
//
BOOST_AUTO_TEST_CASE(decode_0x02_stp_instruction_valid) {

  uint8_t stp_instruction[] = {0x02};

  const auto instruction = jde::decode(stp_instruction, sizeof(stp_instruction));

  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::STP);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x02);

  check_instruction_no_operand(instruction);
}

//
// Test: Disassembles opcode value 0x02.
//
BOOST_AUTO_TEST_CASE(disassemble_0x02_stp_instruction_valid) {

  uint8_t stp_instruction[] = {0x02};

  const auto instructions = jdi::disassemble(stp_instruction, sizeof(stp_instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "STP");
}

//
// Test: Decodes opcode value 0x03 with operand value set to 0xFF.
//
BOOST_AUTO_TEST_CASE(decode_0x03_slo_instruction_valid) {

  uint8_t slo_instruction[] = {0x03, 0xFF};

  const auto instruction = jde::decode(slo_instruction, sizeof(slo_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::SLO);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x03);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x03 with missing operand value.
//
BOOST_AUTO_TEST_CASE(decode_0x03_slo_instruction_invalid_too_small) {

  uint8_t slo_instruction[] = {0x03};

  const auto instruction = jde::decode(slo_instruction, sizeof(slo_instruction));
  check_instruction_invalid(instruction);
}

//
// Test: Disassembles opcode value 0x03.
//
BOOST_AUTO_TEST_CASE(disasemble_0x03_slo_instruction_valid) {

  uint8_t slo_instruction[] = {0x03, 0xFF};

  const auto instructions = jdi::disassemble(slo_instruction, sizeof(slo_instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "SLO ($FF,X)");
}

//
// Test: Decodes opcode value 0x04 with operand value set to 0xFF.
//
BOOST_AUTO_TEST_CASE(decode_0x04_nop_instruction_valid) {

  uint8_t nop_instruction[] = {0x04, 0xFF};

  const auto instruction = jde::decode(nop_instruction, sizeof(nop_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::NOP);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x04);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x04 with missing operand type.
//
BOOST_AUTO_TEST_CASE(decode_0x04_nop_instruction_invalid_too_small) {

  uint8_t nop_instruction[] = {0x04};

  const auto instruction = jde::decode(nop_instruction, sizeof(nop_instruction));
  check_instruction_invalid(instruction);
}

//
// Test: Disassembles opcode value 0x04.
//
BOOST_AUTO_TEST_CASE(disasemble_0x04_nop_instruction_valid) {

  uint8_t instruction[] = {0x04, 0xFF};

  const auto instructions = jdi::disassemble(instruction, sizeof(instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "NOP $FF");
}

//
// Test: Decodes opcode value 0x05 with operand value set to 0xFF
//
BOOST_AUTO_TEST_CASE(decode_0x05_ora_instruction_valid) {

  uint8_t ora_instruction[] = {0x05, 0xFF};

  const auto instruction = jde::decode(ora_instruction, sizeof(ora_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ORA);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x05);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x05 with missing operand value.
//
BOOST_AUTO_TEST_CASE(decode_0x05_ora_instruction_invalid_too_small) {

  uint8_t ora_instruction[] = {0x05};

  const auto instruction = jde::decode(ora_instruction, sizeof(ora_instruction));
  check_instruction_invalid(instruction);
}

//
// Test: Disassembles opcode value 0x05.
//
BOOST_AUTO_TEST_CASE(disasemble_0x05_ora_instruction_valid) {

  uint8_t instruction[] = {0x05, 0xFF};

  const auto instructions = jdi::disassemble(instruction, sizeof(instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "ORA $FF");
}

//
// Test: Decodes opcode value 0x06 with operand value set to 0xFF
//
BOOST_AUTO_TEST_CASE(decode_0x06_asl_instruction_valid) {

  uint8_t asl_instruction[] = {0x06, 0xFF};

  const auto instruction = jde::decode(asl_instruction, sizeof(asl_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ASL);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x06);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x06 with missing operand value.
//
BOOST_AUTO_TEST_CASE(decode_0x06_asl_instruction_invalid_too_small) {

  uint8_t asl_instruction[] = {0x06};

  const auto instruction = jde::decode(asl_instruction, sizeof(asl_instruction));
  check_instruction_invalid(instruction);
}

//
// Test: Disassembles opcode value 0x06.
//
BOOST_AUTO_TEST_CASE(disasemble_0x06_asl_instruction_valid) {

  uint8_t instruction[] = {0x06, 0xFF};

  const auto instructions = jdi::disassemble(instruction, sizeof(instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "ASL $FF");
}

//
// Test: Decodes opcode value 0x07 with operand value set to 0xFF
//
BOOST_AUTO_TEST_CASE(decode_0x07_slo_instruction_valid) {

  uint8_t slo_instruction[] = {0x07, 0xFF};

  const auto instruction = jde::decode(slo_instruction, sizeof(slo_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::SLO);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x07);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x07 with missing operand value.
//
BOOST_AUTO_TEST_CASE(decode_0x07_slo_instruction_invalid_too_small) {

  uint8_t slo_instruction[] = {0x07};

  const auto instruction = jde::decode(slo_instruction, sizeof(slo_instruction));
  check_instruction_invalid(instruction);
}

//
// Test: Disassembles opcode value 0x07.
//
BOOST_AUTO_TEST_CASE(disasemble_0x07_slo_instruction_valid) {

  uint8_t instruction[] = {0x07, 0xFF};

  const auto instructions = jdi::disassemble(instruction, sizeof(instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "SLO $FF");
}

//
// Test: Decodes opcode value 0x08.
//
BOOST_AUTO_TEST_CASE(decode_0x08_php_instruction_valid) {

  uint8_t php_instruction[] = {0x08, 0xFF};

  const auto instruction = jde::decode(php_instruction, sizeof(php_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::PHP);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x08);
  check_instruction_no_operand(instruction);
}

//
// Test: Disassembles opcode value 0x08.
//
BOOST_AUTO_TEST_CASE(disasemble_0x08_php_instruction_valid) {

  uint8_t instruction[] = {0x08};

  const auto instructions = jdi::disassemble(instruction, sizeof(instruction));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "PHP");
}

//
// Test: Decodes opcode value 0x09 with missing operand value.
//
BOOST_AUTO_TEST_CASE(decode_0x09_ora_instruction_invalid_too_small) {

  uint8_t ora_instruction[] = {0x09};

  const auto instruction = jde::decode(ora_instruction, sizeof(ora_instruction));
  check_instruction_invalid(instruction);
}

//
// Test: Decodes opcode value 0x09 with operand set to 0xFF.
//
BOOST_AUTO_TEST_CASE(decode_0x09_ora_instruction_valid) {

  uint8_t ora_instruction[] = {0x09, 0xFF};

  const auto instruction = jde::decode(ora_instruction, sizeof(ora_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ORA);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x09);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::IMMEDIATE);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x0A.
//
BOOST_AUTO_TEST_CASE(decode_0x0A_asl_instruction_valid) {

  uint8_t asl_instruction[] = {0x0A, 0xFF};

  const auto instruction = jde::decode(asl_instruction, sizeof(asl_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ASL);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x0A);
  check_instruction_no_operand(instruction);
}
