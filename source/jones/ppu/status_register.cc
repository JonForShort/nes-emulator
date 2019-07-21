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

auto flag_to_position(status_flag const flag) -> uint8_t {
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

status_register::status_register(uint8_t flags) {
  set(flags);
}

auto status_register::is_set(status_flag const flag) const -> bool {
  return status_flags_.test(flag_to_position(flag));
}

auto status_register::set(status_flag const flag) -> uint8_t {
  status_flags_.set(flag_to_position(flag), true);
  return get();
}

auto status_register::clear(status_flag const flag) -> uint8_t {
  status_flags_.set(flag_to_position(flag), false);
  return get();
}

auto status_register::set(uint8_t const flags) -> uint8_t {
  (0x20U & flags) ? set(status_flag::SPRITE_OVER_FLOW) : clear(status_flag::SPRITE_OVER_FLOW);
  (0x40U & flags) ? set(status_flag::SPRITE_ZERO_HIT) : clear(status_flag::SPRITE_ZERO_HIT);
  (0x80U & flags) ? set(status_flag::VERTICAL_BLANK_STARTED) : clear(status_flag::VERTICAL_BLANK_STARTED);
  return get();
}

auto status_register::get() const -> uint8_t {
  return (is_set(status_flag::SPRITE_OVER_FLOW) ? 0x20U : 0x00U) |
         (is_set(status_flag::SPRITE_ZERO_HIT) ? 0x40U : 0x00U) |
         (is_set(status_flag::VERTICAL_BLANK_STARTED) ? 0x80U : 0x00U);
}