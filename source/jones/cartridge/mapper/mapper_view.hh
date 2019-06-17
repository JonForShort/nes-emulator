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
#ifndef JONES_CARTRIDGE_MAPPERS_MAPPER_VIEW_HH
#define JONES_CARTRIDGE_MAPPERS_MAPPER_VIEW_HH

#include <stdint.h>
#include <string>

namespace jones {

class cartridge;

class memory;

class mapper_view {
public:
  mapper_view(const mapped_cartridge &cartridge, memory &cpu_memory, memory &ppu_memory)
      : cartridge_(cartridge), cpu_memory_(cpu_memory), ppu_memory_(ppu_memory) {}

  ~mapper_view() = default;

  auto cartridge() const -> auto & {
    return cartridge_;
  }

  auto cpu_memory() const -> auto & {
    return cpu_memory_;
  }

  auto ppu_memory() const -> auto & {
    return ppu_memory_;
  }

private:
  const mapped_cartridge &cartridge_;

  memory &cpu_memory_;

  memory &ppu_memory_;
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPERS_MAPPER_VIEW_HH
