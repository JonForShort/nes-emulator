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
#pragma once

#include <cstdint>

//
// CPU reference
// http://e-tradition.net/bytes/6502/6502_instruction_set.html
//
namespace jones {

  using r8_t = uint8_t;

  using r16_t = uint16_t;

  enum class register_t {

    // program counter (16 bit)
    PC,
    
    // accumulator (8 bit)
    AC,

    // X register (8 bit)
    X,

    // Y register (8 bit)
    Y,

    // status register (8 bit)
    SR,
    
    // stack pointer (8 bit)
    SP
  };

  template<register_t R>
  struct register_traits {
    typedef r8_t type;
    uint8_t length = sizeof(type);
  };

  template<>
  struct register_traits<register_t::PC> {
    typedef r16_t type;
    uint8_t length = sizeof(type);
  };
}
