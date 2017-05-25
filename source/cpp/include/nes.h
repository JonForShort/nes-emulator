#pragma once

#include "nes_rom.h"

namespace jones {
  
  class Nes {
  public:
    void load(NesRom nesRom);

    void run();
    
    void reset();
  };
}
