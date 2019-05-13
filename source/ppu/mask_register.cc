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

#include "mask_register.hh"

using namespace jones;

namespace {

uint8_t flag_to_position(const mask_flag flag) {
  switch (flag) {
  case mask_flag::USE_GRAYSCALE:
    return 0;
  case mask_flag::SHOW_LEFT_BACKGROUND:
    return 1;
  case mask_flag::SHOW_LEFT_SPRITES:
    return 2;
  case mask_flag::SHOW_BACKGROUND:
    return 3;
  case mask_flag::SHOW_SPRITES:
    return 4;
  case mask_flag::TINT_RED:
    return 5;
  case mask_flag::TINT_GREEN:
    return 6;
  case mask_flag::TINT_BLUE:
    return 7;
  default:
    BOOST_STATIC_ASSERT("unexpected flag for mask register");
    return -1;
  }
}

} // namespace

bool mask_register::is_set(const mask_flag flag) const {
  return mask_flags_.test(flag_to_position(flag));
}

void mask_register::set(const mask_flag flag) {
  mask_flags_.set(flag_to_position(flag), true);
}

void mask_register::clear(const mask_flag flag) {
  mask_flags_.set(flag_to_position(flag), false);
}

void mask_register::set(const uint8_t flags) {
  (0x01U & flags) ? set(mask_flag::USE_GRAYSCALE) : clear(mask_flag::USE_GRAYSCALE);
  (0x02U & flags) ? set(mask_flag::SHOW_LEFT_BACKGROUND) : clear(mask_flag::SHOW_LEFT_BACKGROUND);
  (0x04U & flags) ? set(mask_flag::SHOW_LEFT_SPRITES) : clear(mask_flag::SHOW_LEFT_SPRITES);
  (0x08U & flags) ? set(mask_flag::SHOW_BACKGROUND) : clear(mask_flag::SHOW_BACKGROUND);
  (0x10U & flags) ? set(mask_flag::SHOW_SPRITES) : clear(mask_flag::SHOW_SPRITES);
  (0x20U & flags) ? set(mask_flag::TINT_RED) : clear(mask_flag::TINT_RED);
  (0x40U & flags) ? set(mask_flag::TINT_GREEN) : clear(mask_flag::TINT_GREEN);
  (0x80U & flags) ? set(mask_flag::TINT_BLUE) : clear(mask_flag::TINT_BLUE);
}

uint8_t mask_register::get() const {
  return (is_set(mask_flag::USE_GRAYSCALE) ? 0x01U : 0x00U) |
         (is_set(mask_flag::SHOW_LEFT_BACKGROUND) ? 0x02U : 0x00U) |
         (is_set(mask_flag::SHOW_LEFT_SPRITES) ? 0x04U : 0x00U) |
         (is_set(mask_flag::SHOW_BACKGROUND) ? 0x08U : 0x00U) |
         (is_set(mask_flag::SHOW_SPRITES) ? 0x10U : 0x00U) |
         (is_set(mask_flag::TINT_RED) ? 0x20U : 0x00U) |
         (is_set(mask_flag::TINT_GREEN) ? 0x40U : 0x00U) |
         (is_set(mask_flag::TINT_BLUE) ? 0x80U : 0x00U);
}