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
#include <cstring>
#include <sstream>
#include <vector>

#include "decode.hh"
#include "disassemble.hh"
#include "instruction.hh"

namespace jdi = jones::disassemble;
namespace jde = jones::decode;

namespace {

std::string disassemble_opcode(const jde::instruction &decoded_instruction) {
  return instruction_set[decoded_instruction.decoded_opcode.value].mnemonic;
}

std::string disassemble_operand_immediate(const jde::operand &decoded_operand) {
  std::stringstream operand_string;
  const int immediate_value = std::get<uint8_t>(decoded_operand.value);
  operand_string << " #$" << std::hex << std::uppercase << immediate_value;
  return operand_string.str();
}

std::string disassemble_operand_memory(const jde::operand &decoded_operand, const addressing_mode_type addressing_mode) {
  std::stringstream operand_string;
  switch (addressing_mode) {
  case addressing_mode_type::ABSOLUTE: {
    const int memory_value = std::get<uint16_t>(decoded_operand.value);
    operand_string << " $" << std::hex << std::uppercase << memory_value;
    break;
  }
  case addressing_mode_type::RELATIVE: {
    const int memory_value = std::get<uint8_t>(decoded_operand.value);
    operand_string << " $" << std::hex << std::uppercase << memory_value;
    break;
  }
  case addressing_mode_type::INDEXED_INDIRECT: {
    const int memory_value = std::get<uint8_t>(decoded_operand.value);
    operand_string << " ($" << std::hex << std::uppercase << memory_value << ",X)";
    break;
  }
  case addressing_mode_type::ZERO_PAGE: {
    const int memory_value = std::get<uint8_t>(decoded_operand.value);
    operand_string << " $" << std::hex << std::uppercase << memory_value;
    break;
  }
  case addressing_mode_type::ZERO_PAGE_X: {
    const int memory_value = std::get<uint8_t>(decoded_operand.value);
    operand_string << " $" << std::hex << std::uppercase << memory_value << ",X";
    break;
  }
  case addressing_mode_type::ZERO_PAGE_Y: {
    const int memory_value = std::get<uint8_t>(decoded_operand.value);
    operand_string << " $" << std::hex << std::uppercase << memory_value << ",Y";
    break;
  }
  }
  return operand_string.str();
}

std::string disassemble_operand(const jde::instruction &decoded_instruction) {
  const auto &decoded_addressing_mode = decoded_instruction.decoded_addressing_mode;
  const auto &decoded_operand = decoded_instruction.decoded_operand;
  switch (decoded_operand.type) {
  case operand_type::IMMEDIATE:
    return disassemble_operand_immediate(decoded_operand);
  case operand_type::MEMORY:
    return disassemble_operand_memory(decoded_operand, decoded_addressing_mode);
  case operand_type::NONE:
    return std::string("");
  default:
    return "";
  }
}

} // namespace

disassemble::instructions disassemble::disassemble(uint8_t *buffer, const size_t length_in_bytes) {
  std::vector<jdi::instruction> disassembled_instructions;
  int buffer_offset = 0;
  auto decoded_instruction = jde::decode(buffer, length_in_bytes);
  while (jde::is_valid(decoded_instruction)) {
    const auto opcode_string = disassemble_opcode(decoded_instruction);
    const auto operand_string = disassemble_operand(decoded_instruction);

    jdi::instruction disassembled_instruction = jdi::instruction();
    disassembled_instruction.address = buffer_offset;
    disassembled_instruction.length_in_bytes = decoded_instruction.encoded_length_in_bytes;
    disassembled_instruction.binary = decoded_instruction.encoded;
    disassembled_instruction.opcode = opcode_string;
    disassembled_instruction.operand = operand_string;
    disassembled_instructions.push_back(disassembled_instruction);

    buffer_offset += decoded_instruction.encoded_length_in_bytes;
    decoded_instruction = jde::decode(buffer + buffer_offset, length_in_bytes - buffer_offset);
  }
  return jdi::instructions{disassembled_instructions, 0};
}
