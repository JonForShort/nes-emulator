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
#ifndef JONES_UTIL_BIT_UTILS_HH
#define JONES_UTIL_BIT_UTILS_HH

#include <cstdint>

namespace jones::util {

typedef uint8_t u8;
typedef int8_t s8;

typedef uint16_t u16;
typedef int16_t s16;

typedef uint32_t u32;
typedef int32_t s32;

typedef uint64_t u64;
typedef int64_t s64;

template <typename T>
constexpr inline auto bit_shift_and(T const data, T const shift, T const bits) -> T {
  return static_cast<T>(data >> shift) & ((1U << bits) - 1);
}

template <typename T>
constexpr inline auto bit_shift_set(T &data, T const shift, T const bits, T const value) -> void {
  data &= ~(((1U << bits) - 1) << shift);
  data |= static_cast<T>(value << shift);
}

template <typename T>
constexpr inline auto bit_reverse(T const value) -> T {
  auto reversed = 0;
  const auto length = sizeof(value) * 8;
  for (size_t index = 0; index < length; index++) {
    const auto b = (value >> (length - index - 1U)) & 0x01;
    reversed |= b << index;
  }
  return reversed;
}

constexpr inline auto bit_value(uint8_t const b) -> uint32_t {
  return 1U << b;
}

} // namespace jones::util

#endif // JONES_UTIL_BIT_UTILS_HH
