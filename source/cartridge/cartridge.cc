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

void printHexInteger(std::ostream &out, const char *label, const uint32_t value) {
  out << "  " << label << std::hex << static_cast<uint32_t>(value) << std::endl;
}

void printHexInteger(std::ostream &out, const char *label, const uint8_t value) {
  out << "  " << label << std::hex << static_cast<uint16_t>(value) << std::endl;
}

} // namespace

Cartridge::Cartridge(const std::string &pathToRom) : romFile_(pathToRom.c_str(), std::ifstream::binary) {
  if (romFile_.good()) {
    romFile_.read((char *)&romFileHeader_, sizeof(romFileHeader_));
  }
}

bool Cartridge::isValid() {
  return romFile_.good() &&
         romFileHeader_.constants == ROM_HEADER_CONSTANT;
}

int Cartridge::getHeaderVersion() const {
  return (romFileHeader_.version == 2) ? 2 : 1;
}

void Cartridge::printHeader(std::ostream &out) const {
  out << "***********************************************" << std::endl;
  out << "  Nes Rom Header" << std::endl;
  out << "***********************************************" << std::endl;
  printHexInteger(out, "constants: ", romFileHeader_.constants);
  printHexInteger(out, "sizeOfPrgRom: ", romFileHeader_.sizeOfPrgRom);
  printHexInteger(out, "sizeOfChrRom: ", romFileHeader_.sizeOfChrRom);
  printHexInteger(out, "hasMirroring: ", romFileHeader_.hasMirroring);
  printHexInteger(out, "containsBatteryBackedPrgRam: ", romFileHeader_.containsBatteryBackedPrgRam);
  printHexInteger(out, "hasTrainer: ", romFileHeader_.hasTrainer);
  printHexInteger(out, "ignoreMirroringControl: ", romFileHeader_.ignoreMirroringControl);
  printHexInteger(out, "lowerMapperNibble: ", romFileHeader_.lowerMapperNibble);
  printHexInteger(out, "isVsUnisystem: ", romFileHeader_.isVsUnisystem);
  printHexInteger(out, "isPlayChoice: ", romFileHeader_.isPlayChoice);
  printHexInteger(out, "version: ", romFileHeader_.version);
  printHexInteger(out, "upperMapperNibble: ", romFileHeader_.upperMapperNibble);
  printHexInteger(out, "sizeOfPrgRam: ", romFileHeader_.sizeOfPrgRam);
  printHexInteger(out, "whichTvSystemOne: ", romFileHeader_.whichTvSystemOne);
  printHexInteger(out, "whichTvSystemTwo: ", romFileHeader_.whichTvSystemTwo);
  printHexInteger(out, "isPrgRamPresent: ", romFileHeader_.isPrgRamPresent);
  printHexInteger(out, "hasBusConflicts: ", romFileHeader_.hasBusConflicts);
  out << "***********************************************" << std::endl;
}
