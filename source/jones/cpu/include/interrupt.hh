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
#ifndef JONES_CPU_INTERRUPT_HH
#define JONES_CPU_INTERRUPT_HH

#include <cstdint>

namespace jones {

enum class interrupt_type : uint8_t {

  // Reset.
  RESET = 0,

  // Non-Maskable.
  NMI,

  // Interrupt Request.
  IRQ,

  // Break.
  BRK,

  // None.
  NONE,

  // Number of Interrupts.
  COUNT
};

enum class interrupt_state : uint8_t {

  // Set Interrupt.
  SET = 0,

  // Clear Interrupt.
  CLEAR,

  // Number of States.
  COUNT
};

} // namespace jones

#endif // JONES_CPU_INTERRUPT_HH
