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
#ifndef JONES_CPU_INTERRUPTS_HH
#define JONES_CPU_INTERRUPTS_HH

#include <array>
#include <boost/static_assert.hpp>

namespace jones {

enum class interrupt_type {

  RESET,

  NMI,

  IRQ,

  COUNT
};

class interrupts {
public:
  bool get(interrupt_type type) {
    switch (type) {
    case interrupt_type::RESET:
      return interrupts_.at(0);
    case interrupt_type::NMI:
      return interrupts_.at(1);
    case interrupt_type::IRQ:
      return interrupts_.at(2);
    default:
      BOOST_STATIC_ASSERT("unexpected interrupt type");
      return false;
    }
  }

  void set(interrupt_type type, bool value) {
    switch (type) {
    case interrupt_type::RESET:
      interrupts_.at(0) = value;
      break;
    case interrupt_type::NMI:
      interrupts_.at(1) = value;
      break;
    case interrupt_type::IRQ:
      interrupts_.at(2) = value;
      break;
    default:
      BOOST_STATIC_ASSERT("unexpected interrupt type");
      break;
    }
  }

private:
  static constexpr size_t interrupt_count = static_cast<size_t>(interrupt_type::COUNT);

  std::array<bool, interrupt_count> interrupts_;
};

} // namespace jones

#endif // JONES_CPU_INTERRUPTS_HH
