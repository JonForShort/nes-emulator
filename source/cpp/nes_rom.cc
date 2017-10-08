#include <iostream>
#include <iomanip>

#include "nes_rom.h"

using namespace jones;

namespace {

  void printHexInteger(std::ostream &out, const char* label, const uint32_t value) {
    out << "  " << label << std::hex << static_cast<uint32_t>(value) << std::endl;
  }

  void printHexInteger(std::ostream &out, const char* label, const uint8_t value) {
    out << "  " << label << std::hex << static_cast<uint16_t>(value) << std::endl;
  }
}

NesRom::NesRom(const std::string& pathToNesRom) :
  m_nesRomFile(pathToNesRom.c_str(), std::ifstream::binary) {
  if (m_nesRomFile.good()) {
    m_nesRomFile.read((char*)&m_nesRomFileHeader, sizeof(NesRomHeader));
  }
}

bool NesRom::isValid() {
  return m_nesRomFile.good() &&
    m_nesRomFileHeader.constants == NES_ROM_HEADER_CONSTANT;
}

int NesRom::getHeaderVersion() const {
  return (m_nesRomFileHeader.version == 2) ? 2 : 1;
}

void NesRom::printHeader(std::ostream &out) const {
  out << "***********************************************" << std::endl;
  out << "  Nes Rom Header"                                << std::endl;
  out << "***********************************************" << std::endl;
  printHexInteger(out, "constants: ", m_nesRomFileHeader.constants);
  printHexInteger(out, "sizeOfPrgRom: ", m_nesRomFileHeader.sizeOfPrgRom);
  printHexInteger(out, "sizeOfChrRom: ", m_nesRomFileHeader.sizeOfChrRom);
  printHexInteger(out, "hasMirroring: ", m_nesRomFileHeader.hasMirroring);
  printHexInteger(out, "containsBatteryBackedPrgRam: ", m_nesRomFileHeader.containsBatteryBackedPrgRam);
  printHexInteger(out, "hasTrainer: ", m_nesRomFileHeader.hasTrainer);
  printHexInteger(out, "ignoreMirroringControl: ", m_nesRomFileHeader.ignoreMirroringControl);
  printHexInteger(out, "lowerMapperNibble: ", m_nesRomFileHeader.lowerMapperNibble);
  printHexInteger(out, "isVsUnisystem: ", m_nesRomFileHeader.isVsUnisystem);
  printHexInteger(out, "isPlayChoice: ", m_nesRomFileHeader.isPlayChoice);
  printHexInteger(out, "version: ", m_nesRomFileHeader.version);
  printHexInteger(out, "upperMapperNibble: ", m_nesRomFileHeader.upperMapperNibble);
  printHexInteger(out, "sizeOfPrgRam: ", m_nesRomFileHeader.sizeOfPrgRam);
  printHexInteger(out, "whichTvSystemOne: ", m_nesRomFileHeader.whichTvSystemOne);
  printHexInteger(out, "whichTvSystemTwo: ", m_nesRomFileHeader.whichTvSystemTwo);
  printHexInteger(out, "isPrgRamPresent: ", m_nesRomFileHeader.isPrgRamPresent);
  printHexInteger(out, "hasBusConflicts: ", m_nesRomFileHeader.hasBusConflicts);
  out << "***********************************************" << std::endl;
}
