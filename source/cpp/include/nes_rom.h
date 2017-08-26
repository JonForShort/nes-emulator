#pragma once

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

    void printHeader(std::ostream &out) const;

  private:
    
    struct NesRomHeader {
      uint32_t constants;
      uint8_t sizeOfPrgRom;
      uint8_t sizeOfChrRom;
      uint8_t hasMirroring : 1;
      uint8_t containsBatteryBackedPrgRam : 1;
      uint8_t hasTrainer : 1;
      uint8_t ignoreMirroringControl : 1;
      uint8_t lowerMapperNibble : 4;
      uint8_t isVsUnisystem : 1;
      uint8_t isPlayChoice : 1;
      uint8_t version : 2;
      uint8_t upperMapperNibble : 4;     
      uint8_t sizeOfPrgRam;
      uint8_t whichTvSystemOne : 1;
      unsigned int : 7;
      uint8_t whichTvSystemTwo : 2;
      unsigned int : 2;
      uint8_t isPrgRamPresent : 1;
      uint8_t hasBusConflicts : 1;
      unsigned int : 2;
      uint32_t ignored;
    };

    NesRomHeader m_nesRomFileHeader;
    
    std::ifstream m_nesRomFile;
  };
}

