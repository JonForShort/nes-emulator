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
#ifndef JONES_CARTRIDGE_MAPPER_MAPPER_CNROM_HH
#define JONES_CARTRIDGE_MAPPER_MAPPER_CNROM_HH

#include <cstdint>
#include <vector>

#include "mapper.hh"

namespace jones {

class mapper_cnrom : public mapper {
public:
  explicit mapper_cnrom(const mapper_view &mapper_view);

  ~mapper_cnrom() override = default;

  [[nodiscard]] auto peek(uint16_t address) const -> uint8_t override;

  [[nodiscard]] auto read(uint16_t address) const -> uint8_t override;

  auto write(uint16_t address, uint8_t data) -> void override;

private:
  auto initialize_chrrom() -> void;

  [[nodiscard]] auto read_prg(uint16_t address) const -> uint8_t;

  [[nodiscard]] auto read_chr(uint16_t address) const -> uint8_t;

  auto write_chr(uint16_t address, uint8_t data) -> void;

  auto write_chr_bank(uint16_t address, uint8_t data) -> void;

  [[nodiscard]] auto read_sram(uint16_t address) const -> uint8_t;

  auto write_sram(uint16_t address, uint8_t data) -> void;

  uint8_t *const prgrom_{};

  uint16_t const prgrom_size_{};

  std::vector<uint8_t> chrom_{};

  uint32_t chr_bank_{};

  std::vector<uint8_t> sram_{};
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPER_MAPPER_CNROM_HH
