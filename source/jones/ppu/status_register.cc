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

using namespace jones::ppu;

namespace {

uint8_t flag_to_position(const status_flag flag) {
  switch (flag) {
  case status_flag::SPRITE_OVER_FLOW:
    return 5;
  case status_flag::SPRITE_ZERO_HIT:
    return 6;
  case status_flag::VERTICAL_BLANK_STARTED:
    return 7;
  default:
    BOOST_STATIC_ASSERT("unexpected flag for status register");
    return -1;
  }
}

} // namespace

bool status_register::is_set(const status_flag flag) const {
  return status_flags_.test(flag_to_position(flag));
}

void status_register::set(const status_flag flag) {
  status_flags_.set(flag_to_position(flag), true);
}

void status_register::clear(const status_flag flag) {
  status_flags_.set(flag_to_position(flag), false);
}

void status_register::set(const uint8_t flags) {
  (0x20U & flags) ? set(status_flag::SPRITE_OVER_FLOW) : clear(status_flag::SPRITE_OVER_FLOW);
  (0x40U & flags) ? set(status_flag::SPRITE_ZERO_HIT) : clear(status_flag::SPRITE_ZERO_HIT);
  (0x80U & flags) ? set(status_flag::VERTICAL_BLANK_STARTED) : clear(status_flag::VERTICAL_BLANK_STARTED);
}

uint8_t status_register::get() const {
  return (is_set(status_flag::SPRITE_OVER_FLOW) ? 0x20U : 0x00U) |
         (is_set(status_flag::SPRITE_ZERO_HIT) ? 0x40U : 0x00U) |
         (is_set(status_flag::VERTICAL_BLANK_STARTED) ? 0x80U : 0x00U);
}