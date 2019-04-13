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

#include <fstream>
#include <stdint.h>
#include <string>

#include "memory.hh"

#define ROM_HEADER_CONSTANT 0x1A53454E

namespace jones {

class cartridge {
public:
  cartridge();

  ~cartridge();

  bool attach(const char *file_path);

  int get_header_version() const;

  void print_header(std::ostream &out) const;

  uint16_t get_prgrom_offset() const;

  uint16_t get_prgrom_size() const;

  uint16_t get_chrrom_offset() const;

  uint16_t get_chrrom_size() const;

  uint8_t get_mapper_number() const;

  uint8_t read(uint16_t address) const;

  void write(uint16_t address, uint8_t data);

private:
  class mapped_cartridge_file;

  struct rom_header {
    uint32_t constants;
    uint8_t prgrom_size;
    uint8_t chrrom_size;
    uint8_t has_mirroring : 1;
    uint8_t contains_battery_backed_prgram : 1;
    uint8_t has_trainer : 1;
    uint8_t ignore_mirroring_control : 1;
    uint8_t lower_mapper_nibble : 4;
    uint8_t is_vs_unisystem : 1;
    uint8_t is_play_choice : 1;
    uint8_t version : 2;
    uint8_t upper_mapper_nibble : 4;
    uint8_t prgram_size;
    uint8_t which_tv_system_one : 1;
    unsigned int : 7;
    uint8_t which_tv_system_two : 2;
    unsigned int : 2;
    uint8_t is_prgram_present : 1;
    uint8_t has_bus_conflicts : 1;
    unsigned int : 2;
    uint8_t ignored[5];
  };

  rom_header rom_file_header_;

  std::unique_ptr<mapped_cartridge_file> cartridge_file_;
};

} // namespace jones

#endif // JONES_CARTRIDGE_CARTRIDGE_HH
