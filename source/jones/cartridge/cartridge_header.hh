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

  auto print(std::ostream &out) const -> void override;

  auto valid() const -> bool override;

  auto version() const -> uint8_t override;

  auto prgrom_offset() const -> uint16_t override;

  auto prgrom_size() const -> uint32_t override;

  auto prgrom_count() const -> uint8_t override;

  auto chrrom_offset() const -> uint16_t override;

  auto chrrom_size() const -> uint16_t override;

  auto chrrom_count() const -> uint8_t override;

  auto mapper_number() const -> uint8_t override;

  auto has_mirroring() const -> bool override;

  auto mirror_type() const -> uint8_t override;
};

} // namespace jones

#endif // JONES_CARTRIDGE_CARTRIDGE_HEADER_HH
