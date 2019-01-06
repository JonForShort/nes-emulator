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
#include <sstream>

#include "addressing_mode.hh"
#include "disassemble.hh"
#include "instruction_set.hh"

using namespace jones;

namespace {

std::string disassemble_opcode(const instruction_t &instruction,
                               uint8_t *buffer, const size_t length_in_bytes) {
  return std::string(instruction.mnemonic);
}

std::string disassemble_operands(const instruction_t &instruction,
                                 uint8_t *buffer,
                                 const size_t length_in_bytes) {
  const auto addressing_mode = instruction.addressing_mode;
  switch (addressing_mode) {
  case addressing_mode_t::IMMEDIATE:
    return "#$2C";
  default:
    return "UNKNOWN";
  }
}
} // namespace

disassemble::instructions
disassemble::disassemble(uint8_t *buffer, const size_t length_in_bytes) {
  if (length_in_bytes < 1) {
    //
    // buffer is not sufficient size to hold an instruction
    //
    return disassemble::instructions{std::vector<std::string>(), 0};
  }
  const auto opcode = buffer[0];
  const auto instruction = instruction_set[opcode];
  if (length_in_bytes < instruction.length) {
    //
    // buffer is smaller than expected size
    //
    return disassemble::instructions{std::vector<std::string>(), 0};
  }
  std::vector<std::string> disassembled_instructions =
      std::vector<std::string>();
  size_t used_length_in_bytes = 0;

  const auto opcode_string =
      disassemble_opcode(instruction, buffer, length_in_bytes);
  const auto operands_string =
      disassemble_operands(instruction, buffer, length_in_bytes);
  disassembled_instructions.emplace_back(opcode_string + " " + operands_string);
  return disassemble::instructions{disassembled_instructions, 0};
}
