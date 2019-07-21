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
#include <fstream>
#include <iostream>

#include "cartridge_header.hh"

using namespace jones;

namespace {

constexpr uint32_t magic_number = 0x1A53454E;

struct formatted_header {
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
} header;

void print_hex_integer(std::ostream &out, const char *label, const uint32_t value) {
  out << "  " << label << std::hex << static_cast<uint32_t>(value) << std::endl;
}

void print_hex_integer(std::ostream &out, const char *label, const uint8_t value) {
  out << "  " << label << std::hex << static_cast<uint16_t>(value) << std::endl;
}

} // namespace

cartridge_header::cartridge_header(std::ifstream file) {
  file.read((char *)&header, sizeof(header));
}

void cartridge_header::print(std::ostream &out) const {
  out << "***********************************************" << std::endl;
  out << "  Nes Rom Header" << std::endl;
  out << "***********************************************" << std::endl;
  print_hex_integer(out, "constants: ", header.constants);
  print_hex_integer(out, "prgrom_size: ", header.prgrom_size);
  print_hex_integer(out, "chrrom_size: ", header.chrrom_size);
  print_hex_integer(out, "has_mirroring: ", header.has_mirroring);
  print_hex_integer(out, "contains_battery_backed_prgram: ", header.contains_battery_backed_prgram);
  print_hex_integer(out, "has_trainer: ", header.has_trainer);
  print_hex_integer(out, "ignore_mirroring_control: ", header.ignore_mirroring_control);
  print_hex_integer(out, "lower_mapper_nibble: ", header.lower_mapper_nibble);
  print_hex_integer(out, "is_vs_unisystem: ", header.is_vs_unisystem);
  print_hex_integer(out, "is_play_choice: ", header.is_play_choice);
  print_hex_integer(out, "version_raw: ", header.version);
  print_hex_integer(out, "version_parsed: ", version());
  print_hex_integer(out, "upper_mapper_nibble: ", header.upper_mapper_nibble);
  print_hex_integer(out, "prgram_size: ", header.prgram_size);
  print_hex_integer(out, "which_tv_system_one: ", header.which_tv_system_one);
  print_hex_integer(out, "which_tv_system_two: ", header.which_tv_system_two);
  print_hex_integer(out, "is_prgram_present: ", header.is_prgram_present);
  print_hex_integer(out, "has_bus_conflicts: ", header.has_bus_conflicts);
  print_hex_integer(out, "mirror_type: ", mirror_type());
  print_hex_integer(out, "mapper_number: ", mapper_number());
  out << "***********************************************" << std::endl;
}

bool cartridge_header::valid() const {
  return header.constants == magic_number;
}

uint8_t cartridge_header::version() const {
  return (header.version == 2) ? 2 : 1;
}

uint16_t cartridge_header::prgrom_offset() const {
  return sizeof(header) + (header.has_trainer == 0 ? 0 : (sizeof(uint8_t) * 512));
}

uint32_t cartridge_header::prgrom_size() const {
  return prgrom_count() * 16384;
}

uint8_t cartridge_header::prgrom_count() const {
  return header.prgrom_size;
}

uint16_t cartridge_header::chrrom_offset() const {
  return prgrom_offset() + prgrom_size();
}

uint16_t cartridge_header::chrrom_size() const {
  return static_cast<uint16_t>(chrrom_count() * 8192);
}

uint8_t cartridge_header::chrrom_count() const {
  return header.chrrom_size;
}

uint8_t cartridge_header::mapper_number() const {
  return (header.upper_mapper_nibble << 4U) | (header.lower_mapper_nibble);
}

bool cartridge_header::has_mirroring() const {
  return header.has_mirroring;
}

auto cartridge_header::mirror_type() const -> uint8_t {
  return header.has_mirroring | (header.ignore_mirroring_control << 1);
}
