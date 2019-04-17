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
#ifndef JONES_CARTRIDGE_MAPPER_MAPPER_NROM_HH
#define JONES_CARTRIDGE_MAPPER_MAPPER_NROM_HH

#include <cstdint>

#include "mapper.hh"

namespace jones {

class mapper_nrom : public mapper {
public:
  explicit mapper_nrom(const cartridge &cartridge);

  ~mapper_nrom() override = default;

  uint8_t read(uint16_t address) override;

  void write(uint16_t address, uint8_t data) override;

private:
  enum class nrom_type {
    NROM_128,
    NROM_256
  };

  enum class nrom_mirroring_type {
    NROM_HORIZONAL,
    NROM_VERTICAL
  };

  static nrom_type resolve_type(const cartridge &cartridge) {
    return cartridge.get_prgrom_size() == 1 ? nrom_type::NROM_128 : nrom_type::NROM_256;
  }

  static nrom_mirroring_type resolve_mirroring_type(const cartridge &cartridge) {
    return cartridge.has_mirroring() ? nrom_mirroring_type::NROM_VERTICAL : nrom_mirroring_type::NROM_HORIZONAL;
  }

  const nrom_type type_;

  const nrom_mirroring_type mirroring_type_;
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPER_MAPPER_NROM_HH
