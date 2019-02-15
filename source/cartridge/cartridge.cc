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
#include <iomanip>
#include <iostream>

#include "cartridge.hh"

using namespace jones;

namespace {

void print_hex_integer(std::ostream &out, const char *label, const uint32_t value) {
  out << "  " << label << std::hex << static_cast<uint32_t>(value) << std::endl;
}

void print_hex_integer(std::ostream &out, const char *label, const uint8_t value) {
  out << "  " << label << std::hex << static_cast<uint16_t>(value) << std::endl;
}

} // namespace

cartridge::cartridge(const std::string &rom_path) : rom_file_(rom_path.c_str(), std::ifstream::binary) {
  if (rom_file_.good()) {
    rom_file_.read((char *)&rom_file_header_, sizeof(rom_file_header_));
  }
}

bool cartridge::is_valid() {
  return rom_file_.good() &&
         rom_file_header_.constants == ROM_HEADER_CONSTANT;
}

int cartridge::get_header_version() const {
  return (rom_file_header_.version == 2) ? 2 : 1;
}

uint16_t cartridge::get_prgrom_offset() const {
  return sizeof(rom_file_header_) + (rom_file_header_.has_trainer == 0 ? 0 : (sizeof(uint8_t) * 512));
}

uint16_t cartridge::get_chrrom_offset() const {
  return get_prgrom_offset() + rom_file_header_.prgrom_size;
}

void cartridge::print_header(std::ostream &out) const {
  out << "***********************************************" << std::endl;
  out << "  Nes Rom Header" << std::endl;
  out << "***********************************************" << std::endl;
  print_hex_integer(out, "constants: ", rom_file_header_.constants);
  print_hex_integer(out, "prgrom_size: ", rom_file_header_.prgrom_size);
  print_hex_integer(out, "chrrom_size: ", rom_file_header_.chrrom_size);
  print_hex_integer(out, "has_mirroring: ", rom_file_header_.has_mirroring);
  print_hex_integer(out, "contains_battery_backed_prgram: ", rom_file_header_.contains_battery_backed_prgram);
  print_hex_integer(out, "has_trainer: ", rom_file_header_.has_trainer);
  print_hex_integer(out, "ignore_mirroring_control: ", rom_file_header_.ignore_mirroring_control);
  print_hex_integer(out, "lower_mapper_nibble: ", rom_file_header_.lower_mapper_nibble);
  print_hex_integer(out, "is_vs_unisystem: ", rom_file_header_.is_vs_unisystem);
  print_hex_integer(out, "is_play_choice: ", rom_file_header_.is_play_choice);
  print_hex_integer(out, "version: ", rom_file_header_.version);
  print_hex_integer(out, "upper_mapper_nibble: ", rom_file_header_.upper_mapper_nibble);
  print_hex_integer(out, "prgram_size: ", rom_file_header_.prgram_size);
  print_hex_integer(out, "which_tv_system_one: ", rom_file_header_.which_tv_system_one);
  print_hex_integer(out, "which_tv_system_two: ", rom_file_header_.which_tv_system_two);
  print_hex_integer(out, "is_prgram_present: ", rom_file_header_.is_prgram_present);
  print_hex_integer(out, "has_bus_conflicts: ", rom_file_header_.has_bus_conflicts);
  out << "***********************************************" << std::endl;
}
