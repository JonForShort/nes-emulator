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
#ifndef JONES_CARTRIDGE_CARTRIDGE_HH
#define JONES_CARTRIDGE_CARTRIDGE_HH

#include "memory.hh"

#include <fstream>
#include <stdint.h>
#include <string>

namespace jones {

class cartridge final {
public:
  cartridge(memory &cpu_memory, memory &ppu_memory);

  ~cartridge();

  auto attach(const char *path) -> bool;

  auto print(std::ostream &out) const -> void;

  auto dump_prg(std::ostream &out) const -> void;

  auto dump_chr(std::ostream &out) const -> void;

  auto peek(uint16_t address) const -> uint8_t;

  auto read(uint16_t address) const -> uint8_t;

  auto write(uint16_t address, uint8_t data) const -> void;

private:
  class impl;

  std::unique_ptr<impl> impl_;
};

} // namespace jones

#endif // JONES_CARTRIDGE_CARTRIDGE_HH
