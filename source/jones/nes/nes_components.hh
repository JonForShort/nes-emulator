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
#ifndef JONES_NES_NES_COMPONENTS_HH
#define JONES_NES_NES_COMPONENTS_HH

#include <cstdint>

namespace jones {

class memory_ram {
public:
  memory_ram() : ram_(ram_size, 0) {}

  ~memory_ram() = default;

  uint8_t read(const uint16_t address) {
    const auto read_address = address % 0x0800U;
    return ram_[read_address];
  }

  void write(const uint16_t address, const uint8_t data) {
    const auto write_address = address % 0x0800U;
    ram_[write_address] = data;
  }

private:
  static constexpr size_t ram_size = 0x0800U;

  std::vector<uint8_t> ram_;
};

} // namespace jones

#endif // JONES_NES_NES_COMPONENTS_HH
