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
#ifndef JONES_CPU_CPU_HH
#define JONES_CPU_CPU_HH

#include <memory>

#include "memory.hh"

namespace jones {

struct cpu_state {

  std::string instruction;

  std::vector<uint8_t> instruction_bytes;

  size_t cycles;

  struct registers {

    uint16_t PC;

    uint8_t A;

    uint8_t X;

    uint8_t Y;

    uint8_t SR;

    uint8_t SP;

  } registers;
};

class cpu final {
public:
  explicit cpu(const memory &memory);

  ~cpu();

  void step();

  void reset();

  uint8_t read(uint16_t address);

  void write(uint16_t address, uint8_t data);

  cpu_state get_state();

  void initialize();

private:
  class impl;

  std::unique_ptr<impl> impl_;
};

} // namespace jones

#endif // JONES_CPU_CPU_HH
