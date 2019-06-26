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
#include <vector>

#include "mapper.hh"

namespace jones {

class mapper_nrom : public mapper {
public:
  mapper_nrom(const mapper_view &mapper_view);

  ~mapper_nrom() override = default;

  auto peek(uint16_t address) const -> uint8_t override;

  auto read(uint16_t address) const -> uint8_t override;

  auto write(uint16_t address, uint8_t data) -> void override;

private:
  auto read_prg(uint16_t address) const -> uint8_t;

  auto write_prg(uint16_t address, uint8_t data) -> void;

  auto read_chr(uint16_t address) const -> uint8_t;

  auto write_chr(uint16_t address, uint8_t data) -> void;

  auto read_sram(uint16_t address) const -> uint8_t;

  auto write_sram(uint16_t address, uint8_t data) -> void;

  enum class nrom_type {
    NROM_128,
    NROM_256
  };

  enum class nrom_mirroring_type {
    NROM_HORIZONAL,
    NROM_VERTICAL
  };

  static nrom_type resolve_type(const mapped_cartridge &cartridge) {
    const auto prgrom_count = cartridge.header()->prgrom_count();
    return prgrom_count == 1 ? nrom_type::NROM_128 : nrom_type::NROM_256;
  }

  static nrom_mirroring_type resolve_mirroring_type(const mapped_cartridge &cartridge) {
    const auto mirroring = cartridge.header()->has_mirroring();
    return mirroring ? nrom_mirroring_type::NROM_VERTICAL : nrom_mirroring_type::NROM_HORIZONAL;
  }

  const nrom_type type_{};

  const nrom_mirroring_type mirroring_type_{};

  const bool use_chrram{};

  uint8_t *const prgrom_{};

  const uint16_t prgrom_size_{};

  std::vector<uint8_t> chrram_{};

  std::vector<uint8_t> sram_{};
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPER_MAPPER_NROM_HH
