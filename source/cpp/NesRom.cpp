#include <iostream>
#include <iomanip>

#include "include/NesRom.h"

using namespace jones;

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
