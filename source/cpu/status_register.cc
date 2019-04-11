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
#include <boost/static_assert.hpp>

#include "status_register.hh"

using namespace jones;

bool status_register::is_set(status_flag flag) {
  return status_flags_.test(flag_to_position(flag));
}

void status_register::set(status_flag flag) {
  status_flags_.set(flag_to_position(flag), true);
}

void status_register::clear(status_flag flag) {
  status_flags_.set(flag_to_position(flag), false);
}

uint8_t status_register::flag_to_position(status_flag flag) {
  switch (flag) {
  case status_flag::N:
    return 7;
  case status_flag::V:
    return 6;
  case status_flag::B1:
    return 5;
  case status_flag::B2:
    return 4;
  case status_flag::D:
    return 3;
  case status_flag::I:
    return 2;
  case status_flag::Z:
    return 1;
  case status_flag::C:
    return 0;
  default:
    BOOST_STATIC_ASSERT("missing status flag");
    return 0;
  }
}
