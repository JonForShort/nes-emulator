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
#ifndef JONES_CARTRIDGE_MAPPER_MAPPER_HH
#define JONES_CARTRIDGE_MAPPER_MAPPER_HH

#include <cstdint>
#include <memory>

#include "cartridge.hh"

namespace jones {

class mapper {
public:
  explicit mapper(const cartridge &cartridge) : cartridge_(cartridge) {}

  virtual ~mapper() = default;

  virtual uint8_t read(uint16_t address) = 0;

  virtual void write(uint16_t address, uint8_t data) = 0;

protected:
  const cartridge &get_cartridge() {
    return cartridge_;
  }

private:
  const cartridge &cartridge_;
};

class mappers {
public:
  static std::unique_ptr<mapper> get(const cartridge &cartridge, uint16_t mapper_number);
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPER_MAPPER_HH