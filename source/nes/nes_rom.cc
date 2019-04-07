//
// MIT License
//
// Copyright 2017-2019
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

#include "nes_rom.hh"

using namespace jones;

namespace {

void print_hex_integer(std::ostream &out, const char *label, const uint32_t value) {
  out << "  " << label << std::hex << static_cast<uint32_t>(value) << std::endl;
}

void print_hex_integer(std::ostream &out, const char *label, const uint8_t value) {
  out << "  " << label << std::hex << static_cast<uint16_t>(value) << std::endl;
}

} // namespace

nes_rom::nes_rom(const std::string &rom_path)
    : rom_file_header_(), rom_file_(rom_path.c_str(), std::ifstream::binary) {
  if (rom_file_.good()) {
    rom_file_.read((char *)&rom_file_header_, sizeof(nes_rom_header));
  }
}

bool nes_rom::is_valid() const {
  return rom_file_.good() &&
         rom_file_header_.constants == NES_ROM_HEADER_CONSTANT;
}

int nes_rom::header_version() const {
  return (rom_file_header_.version == 2) ? 2 : 1;
}

void nes_rom::print_header(std::ostream &out) const {
  out << "***********************************************" << std::endl;
  out << "  Nes Rom Header" << std::endl;
  out << "***********************************************" << std::endl;
  print_hex_integer(out, "constants: ", rom_file_header_.constants);
  print_hex_integer(out, "sizeOfPrgRom: ", rom_file_header_.sizeOfPrgRom);
  print_hex_integer(out, "sizeOfChrRom: ", rom_file_header_.sizeOfChrRom);
  print_hex_integer(out, "hasMirroring: ", rom_file_header_.hasMirroring);
  print_hex_integer(out, "containsBatteryBackedPrgRam: ", rom_file_header_.containsBatteryBackedPrgRam);
  print_hex_integer(out, "hasTrainer: ", rom_file_header_.hasTrainer);
  print_hex_integer(out, "ignoreMirroringControl: ", rom_file_header_.ignoreMirroringControl);
  print_hex_integer(out, "lowerMapperNibble: ", rom_file_header_.lowerMapperNibble);
  print_hex_integer(out, "isVsUnisystem: ", rom_file_header_.isVsUnisystem);
  print_hex_integer(out, "isPlayChoice: ", rom_file_header_.isPlayChoice);
  print_hex_integer(out, "version: ", rom_file_header_.version);
  print_hex_integer(out, "upperMapperNibble: ", rom_file_header_.upperMapperNibble);
  print_hex_integer(out, "sizeOfPrgRam: ", rom_file_header_.sizeOfPrgRam);
  print_hex_integer(out, "whichTvSystemOne: ", rom_file_header_.whichTvSystemOne);
  print_hex_integer(out, "whichTvSystemTwo: ", rom_file_header_.whichTvSystemTwo);
  print_hex_integer(out, "isPrgRamPresent: ", rom_file_header_.isPrgRamPresent);
  print_hex_integer(out, "hasBusConflicts: ", rom_file_header_.hasBusConflicts);
  out << "***********************************************" << std::endl;
}
