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

void check_disassemble(uint8_t *buffer, size_t length_in_bytes, const char *expected) {
  const auto instructions = jdi::disassemble(buffer, length_in_bytes);
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  const auto disassembled = first_instruction.opcode + first_instruction.operand;
  BOOST_CHECK(disassembled == expected);
}

} // namespace

BOOST_AUTO_TEST_CASE(cpu_test) { BOOST_CHECK(true); }

//
// Test: Disassembles opcode 0x00.
//
BOOST_AUTO_TEST_CASE(disassemble_0x00_instruction_valid) {
  uint8_t binary_instruction[] = {0x00};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "BRK");
}

//
// Test: Disassembles opcode 0x01.
//
BOOST_AUTO_TEST_CASE(disassemble_0x01_instruction_valid) {
  uint8_t binary_instruction[] = {0x01, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA ($FF,X)");
}

//
// Test: Disassembles opcode 0x02.
//
BOOST_AUTO_TEST_CASE(disassemble_0x02_instruction_valid) {
  uint8_t binary_instruction[] = {0x02};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "STP");
}

//
// Test: Disassembles opcode 0x03.
//
BOOST_AUTO_TEST_CASE(disassemble_0x03_slo_instruction_valid) {
  uint8_t binary_instruction[] = {0x03, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "SLO ($FF,X)");
}

//
// Test: Disassembles opcode 0x04.
//
BOOST_AUTO_TEST_CASE(disassemble_0x04_instruction_valid) {
  uint8_t binary_instruction[] = {0x04, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "NOP $FF");
}

//
// Test: Disassembles opcode 0x05.
//
BOOST_AUTO_TEST_CASE(disassemble_0x05_instruction_valid) {
  uint8_t binary_instruction[] = {0x05, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA $FF");
}

//
// Test: Disassembles opcode 0x06.
//
BOOST_AUTO_TEST_CASE(disassemble_0x06_instruction_valid) {
  uint8_t binary_instruction[] = {0x06, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ASL $FF");
}

//
// Test: Disassembles opcode 0x07.
//
BOOST_AUTO_TEST_CASE(disassemble_0x07_instruction_valid) {
  uint8_t binary_instruction[] = {0x07, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "SLO $FF");
}

//
// Test: Disassembles opcode 0x08.
//
BOOST_AUTO_TEST_CASE(disassemble_0x08_instruction_valid) {
  uint8_t binary_instruction[] = {0x08};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "PHP");
}

//
// Test: Disassembles opcode 0x09.
//
BOOST_AUTO_TEST_CASE(disassemble_0x09_instruction_valid) {
  uint8_t binary_instruction[] = {0x09, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA #$FF");
}

//
// Test: Disassembles opcode 0x0A.
//
BOOST_AUTO_TEST_CASE(disassemble_0x0A_instruction_valid) {
  uint8_t binary_instruction[] = {0x0A};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ASL");
}

//
// Test: Disassembles opcode 0x0B.
//
BOOST_AUTO_TEST_CASE(disassemble_0x0B_instruction_valid) {
  uint8_t binary_instruction[] = {0x0B, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ANC #$FF");
}

//
// Test: Disassembles opcode 0x0C.
//
BOOST_AUTO_TEST_CASE(disassemble_0x0C_instruction_valid) {
  uint8_t binary_instruction[] = {0x0C, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "NOP $EEFF");
}

//
// Test: Disassembles opcode 0x0D.
//
BOOST_AUTO_TEST_CASE(disassemble_0x0D_instruction_valid) {
  uint8_t binary_instruction[] = {0x0D, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA $EEFF");
}

//
// Test: Disassembles opcode 0x0E.
//
BOOST_AUTO_TEST_CASE(disassemble_0x0E_instruction_valid) {
  uint8_t binary_instruction[] = {0x0E, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ASL $EEFF");
}

//
// Test: Disassembles opcode 0x0F.
//
BOOST_AUTO_TEST_CASE(disassemble_0x0F_instruction_valid) {
  uint8_t binary_instruction[] = {0x0F, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "SLO $EEFF");
}

//
// Test: Disassembles opcode 0x10.
//
BOOST_AUTO_TEST_CASE(disassemble_0x10_instruction_valid) {
  uint8_t binary_instruction[] = {0x10, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "BPL $FF");
}

//
// Test: Disassembles opcode 0x11.
//
BOOST_AUTO_TEST_CASE(disassemble_0x11_instruction_valid) {
  uint8_t binary_instruction[] = {0x11, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA ($FF),Y");
}

//
// Test: Disassembles opcode 0x12.
//
BOOST_AUTO_TEST_CASE(disassemble_0x12_instruction_valid) {
  uint8_t binary_instruction[] = {0x12};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "STP");
}

//
// Test: Disassembles opcode 0x13.
//
BOOST_AUTO_TEST_CASE(disassemble_0x13_instruction_valid) {
  uint8_t binary_instruction[] = {0x13, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "SLO ($FF),Y");
}

//
// Test: Disassembles opcode 0x14.
//
BOOST_AUTO_TEST_CASE(disassemble_0x14_instruction_valid) {
  uint8_t binary_instruction[] = {0x14, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "NOP $FF,X");
}

//
// Test: Disassembles opcode 0x15.
//
BOOST_AUTO_TEST_CASE(disassemble_0x15_instruction_valid) {
  uint8_t binary_instruction[] = {0x15, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA $FF,X");
}

//
// Test: Disassembles opcode 0x16.
//
BOOST_AUTO_TEST_CASE(disassemble_0x16_instruction_valid) {
  uint8_t binary_instruction[] = {0x16, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ASL $FF,X");
}

//
// Test: Disassembles opcode 0x17.
//
BOOST_AUTO_TEST_CASE(disassemble_0x17_instruction_valid) {
  uint8_t binary_instruction[] = {0x17, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "SLO $FF,X");
}

//
// Test: Disassembles opcode 0x18.
//
BOOST_AUTO_TEST_CASE(disassemble_0x18_instruction_valid) {
  uint8_t binary_instruction[] = {0x18};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "CLC");
}

//
// Test: Disassembles opcode 0x19.
//
BOOST_AUTO_TEST_CASE(disassemble_0x19_instruction_valid) {
  uint8_t binary_instruction[] = {0x19, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA $EEFF,Y");
}

//
// Test: Disassembles opcode 0x1A.
//
BOOST_AUTO_TEST_CASE(disassemble_0x1A_instruction_valid) {
  uint8_t binary_instruction[] = {0x1A};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "NOP");
}

//
// Test: Disassembles opcode 0x1B.
//
BOOST_AUTO_TEST_CASE(disassemble_0x1B_instruction_valid) {
  uint8_t binary_instruction[] = {0x1B, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "SLO $EEFF,Y");
}

//
// Test: Disassembles opcode 0x1C.
//
BOOST_AUTO_TEST_CASE(disassemble_0x1C_instruction_valid) {
  uint8_t binary_instruction[] = {0x1C, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "NOP $EEFF,X");
}

//
// Test: Disassembles opcode 0x1D.
//
BOOST_AUTO_TEST_CASE(disassemble_0x1D_instruction_valid) {
  uint8_t binary_instruction[] = {0x1D, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ORA $EEFF,X");
}

//
// Test: Disassembles opcode 0x1E.
//
BOOST_AUTO_TEST_CASE(disassemble_0x1E_instruction_valid) {
  uint8_t binary_instruction[] = {0x1E, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ASL $EEFF,X");
}

//
// Test: Disassembles opcode 0x1F.
//
BOOST_AUTO_TEST_CASE(disassemble_0x1F_instruction_valid) {
  uint8_t binary_instruction[] = {0x1F, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "SLO $EEFF,X");
}

//
// Test: Disassembles opcode 0x20.
//
BOOST_AUTO_TEST_CASE(disassemble_0x20_instruction_valid) {
  uint8_t binary_instruction[] = {0x20, 0xFF, 0xEE};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "JSR $EEFF");
}

//
// Test: Disassembles opcode 0x21.
//
BOOST_AUTO_TEST_CASE(disassemble_0x21_instruction_valid) {
  uint8_t binary_instruction[] = {0x21, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "AND ($FF,X)");
}

//
// Test: Disassembles opcode 0x22.
//
BOOST_AUTO_TEST_CASE(disassemble_0x22_instruction_valid) {
  uint8_t binary_instruction[] = {0x22};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "STP");
}

//
// Test: Disassembles opcode 0x23.
//
BOOST_AUTO_TEST_CASE(disassemble_0x23_instruction_valid) {
  uint8_t binary_instruction[] = {0x23, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "RLA ($FF,X)");
}

//
// Test: Disassembles opcode 0x24.
//
BOOST_AUTO_TEST_CASE(disassemble_0x24_instruction_valid) {
  uint8_t binary_instruction[] = {0x24, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "BIT $FF");
}

//
// Test: Disassembles opcode 0x25.
//
BOOST_AUTO_TEST_CASE(disassemble_0x25_instruction_valid) {
  uint8_t binary_instruction[] = {0x25, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "AND $FF");
}

//
// Test: Disassembles opcode 0x26.
//
BOOST_AUTO_TEST_CASE(disassemble_0x26_instruction_valid) {
  uint8_t binary_instruction[] = {0x26, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ROL $FF");
}

//
// Test: Disassembles opcode 0x27.
//
BOOST_AUTO_TEST_CASE(disassemble_0x27_instruction_valid) {
  uint8_t binary_instruction[] = {0x27, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "RLA $FF");
}

//
// Test: Disassembles opcode 0x28.
//
BOOST_AUTO_TEST_CASE(disassemble_0x28_instruction_valid) {
  uint8_t binary_instruction[] = {0x28, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "PLP");
}

//
// Test: Disassembles opcode 0x29.
//
BOOST_AUTO_TEST_CASE(disassemble_0x29_instruction_valid) {
  uint8_t binary_instruction[] = {0x29, 0x2C};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "AND #$2C");
}

//
// Test: Disassembles opcode 0x2A.
//
BOOST_AUTO_TEST_CASE(disassemble_0x2A_instruction_valid) {
  uint8_t binary_instruction[] = {0x2A};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ROL");
}

//
// Test: Disassembles opcode 0x2B.
//
BOOST_AUTO_TEST_CASE(disassemble_0x2B_instruction_valid) {
  uint8_t binary_instruction[] = {0x2B, 0x2C};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ANC #$2C");
}

//
// Test: Disassembles opcode 0x2C.
//
BOOST_AUTO_TEST_CASE(disassemble_0x2C_instruction_valid) {
  uint8_t binary_instruction[] = {0x2C, 0xEE, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "BIT $FFEE");
}

//
// Test: Disassembles opcode 0x2D.
//
BOOST_AUTO_TEST_CASE(disassemble_0x2D_instruction_valid) {
  uint8_t binary_instruction[] = {0x2D, 0xEE, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "AND $FFEE");
}

//
// Test: Disassembles opcode 0x2E.
//
BOOST_AUTO_TEST_CASE(disassemble_0x2E_instruction_valid) {
  uint8_t binary_instruction[] = {0x2E, 0xEE, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "ROL $FFEE");
}

//
// Test: Disassembles opcode 0x2F.
//
BOOST_AUTO_TEST_CASE(disassemble_0x2F_instruction_valid) {
  uint8_t binary_instruction[] = {0x2F, 0xEE, 0xFF};
  check_disassemble(binary_instruction, sizeof(binary_instruction), "RLA $FFEE");
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
  uint8_t asl_instruction[] = {0x0A};
  const auto instruction = jde::decode(asl_instruction, sizeof(asl_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ASL);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x0A);
  check_instruction_no_operand(instruction);
}

//
// Test: Decodes opcode value 0x0B.
//
BOOST_AUTO_TEST_CASE(decode_0x0B_anc_instruction_valid) {
  uint8_t anc_instruction[] = {0x0B, 0xFF};
  const auto instruction = jde::decode(anc_instruction, sizeof(anc_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ANC);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x0B);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::IMMEDIATE);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x0C.
//
BOOST_AUTO_TEST_CASE(decode_0x0C_nop_instruction_valid) {
  uint8_t nop_instruction[] = {0x0C, 0xFF, 0xEE};
  const auto instruction = jde::decode(nop_instruction, sizeof(nop_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::NOP);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x0C);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint16_t>(instruction.decoded_operand.value) == static_cast<uint16_t>(0xEEFF));
}

//
// Test: Decodes opcode value 0x0D.
//
BOOST_AUTO_TEST_CASE(decode_0x0D_ora_instruction_valid) {
  uint8_t binary_instruction[] = {0x0D, 0xFF, 0xEE};
  const auto instruction = jde::decode(binary_instruction, sizeof(binary_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ORA);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x0D);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint16_t>(instruction.decoded_operand.value) == static_cast<uint16_t>(0xEEFF));
}

//
// Test: Decodes opcode value 0x0E.
//
BOOST_AUTO_TEST_CASE(decode_0x0E_asl_instruction_valid) {
  uint8_t binary_instruction[] = {0x0E, 0xFF, 0xEE};
  const auto instruction = jde::decode(binary_instruction, sizeof(binary_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ASL);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x0E);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint16_t>(instruction.decoded_operand.value) == static_cast<uint16_t>(0xEEFF));
}

//
// Test: Decodes opcode value 0x0F.
//
BOOST_AUTO_TEST_CASE(decode_0x0F_instruction_valid) {
  uint8_t binary_instruction[] = {0x0F, 0xFF, 0xEE};
  const auto instruction = jde::decode(binary_instruction, sizeof(binary_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::SLO);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x0F);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint16_t>(instruction.decoded_operand.value) == static_cast<uint16_t>(0xEEFF));
}

//
// Test: Decodes opcode value 0x10.
//
BOOST_AUTO_TEST_CASE(decode_0x10_instruction_valid) {
  uint8_t binary_instruction[] = {0x10, 0xFF};
  const auto instruction = jde::decode(binary_instruction, sizeof(binary_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::BPL);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x10);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x11.
//
BOOST_AUTO_TEST_CASE(decode_0x11_instruction_valid) {
  uint8_t binary_instruction[] = {0x11, 0xFF};
  const auto instruction = jde::decode(binary_instruction, sizeof(binary_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::ORA);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x11);
  BOOST_CHECK(instruction.decoded_operand.type == operand_type::MEMORY);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0xFF));
}

//
// Test: Decodes opcode value 0x12.
//
BOOST_AUTO_TEST_CASE(decode_0x12_instruction_valid) {
  uint8_t binary_instruction[] = {0x12};
  const auto instruction = jde::decode(binary_instruction, sizeof(binary_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::STP);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x12);
  check_instruction_no_operand(instruction);
}
