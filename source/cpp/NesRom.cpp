#include <iostream>
#include <iomanip>

#include "include/NesRom.h"

using namespace jones;

NesRom::NesRom(const std::string& pathToNesRom) :
  m_nesRomFile(pathToNesRom.c_str(), std::ifstream::binary) {
}

bool NesRom::isValid() {
  if (!m_nesRomFile.good()) {
    std::cout << "nes rom file is not good" << std::endl;
    return false;
  }

  Header header;
  m_nesRomFile.read((char*)&header, sizeof(Header));
  if (header.constants != NES_ROM_HEADER_CONSTANT) {
    std::cout << "nes rom file header constants do not match" << std::endl;
    return false;
  }

  return true;
}
