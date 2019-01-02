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
#include "instruction_set.hh"
#include "opcode.hh"
#include "register.hh"
#include "status_register_flag.hh"

using namespace jones;

class cpu::cpu_impl final {
public:
  cpu_impl(const void* base_address) : base_address_(base_address) {}

  void step() {
  }

  void reset() {
  }

  void run() {
  }

private:
  const void* base_address_;
};

cpu::cpu(const void* base_address) : impl_(new cpu_impl(base_address)) {}

cpu::~cpu() = default;

cpu::cpu(cpu &&) noexcept = default;

cpu& cpu::operator=(cpu &&) noexcept = default;

void cpu::step() {
  impl_->step();
}

void cpu::reset() {
  impl_->reset();
}

void cpu::run() {
  impl_->run();
}
