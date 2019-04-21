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
#ifndef JONES_CARTRIDGE_CARTRIDGE_HEADER_HH
#define JONES_CARTRIDGE_CARTRIDGE_HEADER_HH

#include "mapper/mapped_cartridge.hh"

#include <stdint.h>
#include <string>

namespace jones {

class cartridge_header final : public mapped_cartridge_header {
public:
  explicit cartridge_header(std::ifstream file);

  ~cartridge_header() override = default;

  void print(std::ostream &out) const override;

  bool valid() const override;

  size_t version() const override;

  uint16_t prgrom_offset() const override;

  uint16_t prgrom_size() const override;

  uint8_t prgrom_count() const override;

  uint16_t chrrom_offset() const override;

  uint16_t chrrom_size() const override;

  uint8_t chrrom_count() const override;

  uint8_t mapper_number() const override;

  bool mirroring() const override;
};

} // namespace jones

#endif // JONES_CARTRIDGE_CARTRIDGE_HEADER_HH
