//
// MIT License
//
// Copyright 2017-2018
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
#include "cpu.hh"
#include "opcode_table.hh"
#include "registers.hh"
#include "status_register_flags.hh"
#include "instruction_set.hh"

using namespace jones;

class Cpu::CpuImpl final {
public:
  CpuImpl(const void* baseAddress) {

  }

  void step() {

  }

  void reset() {

  }

  void run() {

  }

private:
  const void* baseAddress_;
};

Cpu::Cpu(const void* baseAddress) : impl_(new CpuImpl(baseAddress)) {}

Cpu::~Cpu() = default;

Cpu::Cpu(Cpu &&) noexcept = default;

Cpu& Cpu::operator=(Cpu &&) noexcept = default;

void Cpu::step() {
  impl_->step();
}

void Cpu::reset() {
  impl_->reset();
}

void Cpu::run() {
  impl_->run();
}
