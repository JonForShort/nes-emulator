//
// MIT License
//
// Copyright 2018
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

//
// Reference:
// https://wiki.nesdev.com/w/index.php/CPU_addressing_modes
//
namespace jones {

enum class addressing_mode_t {

  // d : Zero Page Indexed
  ZERO_PAGE,

  // d,x : Zero Page Indexed with X
  ZERO_PAGE_X,

  // d,y : Zero Page Indexed with Y
  ZERO_PAGE_Y,

  // a : Absolute Indexed
  ABSOLUTE,

  // a,x : Absolute Indexed with X
  ABSOLUTE_X,

  // a,y : Absolute Indexed with Y
  ABSOLUTE_Y,

  // (d,x) : Indexed Indirect
  INDEXED_INDIRECT,

  // (d),y : Indirect Indexed
  INDIRECT_INDEXED,

  // () : Implicit
  IMPLICIT,

  // A : Accumulator
  ACCUMULATOR,

  // #v : Immediate
  IMMEDIATE,

  // +d : Relative
  RELATIVE,

  // (a) : Indirect
  INDIRECT
};

} // namespace jones
