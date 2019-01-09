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
#include "decode.hh"
#include <cstring>

using namespace jones;

decode::instruction decode::decode(uint8_t *buffer, size_t length_in_bytes) {
  if (length_in_bytes < 1) {
    //
    // buffer is not sufficient size to hold an instruction
    //
    return decode::instruction();
  }
  const auto opcode = buffer[0];
  const auto instruction = instruction_set[opcode];
  if (length_in_bytes < instruction.length) {
    //
    // buffer is smaller than expected size
    //
    return decode::instruction();
  }
  const auto instruction_length = instruction.length;
  decode::instruction decoded_instruction = {0};
  std::memcpy(decoded_instruction.encoded, buffer, instruction_length);
  decoded_instruction.encoded_length_in_bytes = instruction_length;

  decoded_instruction.decoded_opcode = decode::opcode{instruction.opcode};
  
  return decoded_instruction;
}
