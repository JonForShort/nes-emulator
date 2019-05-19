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
#ifndef JONES_CPU_DISASSEMBLE_HH
#define JONES_CPU_DISASSEMBLE_HH

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace jones::disassemble {

constexpr unsigned int max_length_in_bytes = 3;

struct instruction {
  uint16_t address;
  uint8_t length_in_bytes;
  std::array<uint8_t, max_length_in_bytes> binary;
  std::string opcode;
  std::string operand;
};

struct instructions {
  std::vector<instruction> instructions;
  size_t total_length_in_bytes;
};

instructions disassemble(uint8_t *buffer, size_t length_in_bytes);

} // namespace jones::disassemble

#endif // JONES_CPU_DISASSEMBLE_HH
