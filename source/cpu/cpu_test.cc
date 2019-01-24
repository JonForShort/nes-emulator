//
// MIT License
//
// Copyright 2018
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

BOOST_AUTO_TEST_CASE(cpu_test) { BOOST_CHECK(true); }

BOOST_AUTO_TEST_CASE(disasemble_test_and_immediate) {

  uint8_t and_immediate[] = {0x29, 0x2C};

  const auto instructions = jdi::disassemble(and_immediate, sizeof(and_immediate));
  BOOST_CHECK(instructions.instructions.size() == 1);

  const auto &first_instruction = instructions.instructions[0];
  BOOST_CHECK(first_instruction == "AND #$2C");
}

BOOST_AUTO_TEST_CASE(decode_invalid_instruction_too_small) {

  uint8_t empty_instruction[] = {};

  const auto instruction = jde::decode(empty_instruction, sizeof(empty_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::INVALID);
}

BOOST_AUTO_TEST_CASE(decode_brk_instruction_valid) {

  uint8_t brk_instruction[] = {0x00};

  const auto instruction = jde::decode(brk_instruction, sizeof(brk_instruction));
  BOOST_CHECK(instruction.decoded_opcode.type == opcode_type::BRK);
  BOOST_CHECK(instruction.decoded_opcode.value == 0x00);

  BOOST_CHECK(instruction.decoded_operand.type == operand_type::NONE);
  BOOST_CHECK(std::get<uint8_t>(instruction.decoded_operand.value) == static_cast<uint8_t>(0x00));
}
