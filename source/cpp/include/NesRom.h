#ifndef INCLUDE_NES_ROM_H
#define INCLUDE_NES_ROM_H

#include <string>
#include <fstream>
#include <stdint.h>

#define NES_ROM_HEADER_CONSTANT 0x1A53454E

namespace jones {

  class NesRom {
    
  public:

    NesRom(const std::string& pathToNesRom);

    bool isValid();

    int getHeaderVersion() const;
    
  private:
    
    struct NesRomHeader {
      uint32_t constants;
      uint8_t sizeOfPrgRom;
      uint8_t sizeOfChrRom;
      uint8_t firstSetFlags;
      uint8_t secondSetFlags;
      uint8_t sizeOfPrgRam;
      uint8_t thirdSetFlags;
      uint8_t fourthSetFlags;
      uint32_t ignored;
    };

    NesRomHeader m_nesRomFileHeader;
    
    std::ifstream m_nesRomFile;
  };
}

#endif // INCLUDE_NES_ROM_H
