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
#ifndef JONES_CARTRIDGE_MAPPER_MAPPER_MMC1_HH
#define JONES_CARTRIDGE_MAPPER_MAPPER_MMC1_HH

#include <cstdint>
#include <vector>

#include "mapper.hh"

namespace jones {

class mapper_mmc1 : public mapper {
public:
  explicit mapper_mmc1(const mapper_view &mapper_view);

  ~mapper_mmc1() override = default;

  auto peek(uint16_t address) const -> uint8_t override;

  auto read(uint16_t address) const -> uint8_t override;

  auto write(uint16_t address, uint8_t data) -> void override;

private:
  auto get_prg_bank_offset(int16_t index) const -> uint16_t;

  auto get_chr_bank_offset(int16_t index) const -> uint16_t;

  auto update_offsets() -> void;

  auto load_register(uint16_t address, uint8_t data) -> void;

  auto write_register(uint16_t address, uint8_t data) -> void;

  auto write_control_register(uint8_t data) -> void;

  auto write_chr_bank_zero(uint8_t data) -> void;

  auto write_chr_bank_one(uint8_t data) -> void;

  auto write_prg_bank(uint8_t data) -> void;

private:
  uint8_t shift_register_{};

  uint8_t control_register_{};

  uint8_t prg_mode_{};

  uint8_t chr_mode_{};

  uint8_t prg_bank_{};

  uint8_t chr_bank_[2]{};

  uint32_t prg_offsets_[2]{};

  uint32_t chr_offsets_[2]{};

  uint8_t *prg_rom_{};

  uint16_t prg_rom_size_{};

  uint8_t *chr_rom_{};

  uint16_t chr_rom_size_{};

  std::vector<uint8_t> chr_ram_{};

  std::vector<uint8_t> sram_{};
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPER_MAPPER_MMC1_HH
