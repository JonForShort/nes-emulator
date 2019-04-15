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
#ifndef JONES_CPU_STATUS_REGISTER_HH
#define JONES_CPU_STATUS_REGISTER_HH

#include <bitset>
#include <cstdint>

namespace jones {

//
// https://wiki.nesdev.com/w/index.php/Status_flags
//
enum class status_flag {

  // Negative
  N = 0,

  // Overflow
  V,

  // Ignored
  B1,

  // Ignored
  B2,

  // Decimal
  D,

  // Interrupt Disable
  I,

  // Zero
  Z,

  // Carry
  C,

  // Number of flags
  COUNT
};

class status_register final {
public:
  bool is_set(status_flag flag) const;

  void set(status_flag flag);

  void clear(status_flag flag);

  void set(uint8_t flags);

  uint8_t get() const;

private:
  static uint8_t flag_to_position(status_flag flag);

private:
  static constexpr size_t status_flag_count = static_cast<size_t>(status_flag::COUNT);

  std::bitset<status_flag_count> status_flags_;
};

} // namespace jones

#endif // JONES_CPU_STATUS_REGISTER_HH
