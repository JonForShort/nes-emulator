#include <iostream>

#include "nes_rom.h"
#include "screen.h"

int main(int argc, char *argv[]){

  if (argc != 2) {
    std::cout << "must specify path to nes rom file" << std::endl;
    return -1;
  }

  jones::NesRom nesRom(argv[1]);
  if (!nesRom.isValid()) {
    std::cout << "must specify valid nes rom file" << std::endl;
    return -2;
  }

  nesRom.printHeader(std::cout);

  jones::Screen screen;
  screen.initialize();
  screen.release();

  return 0;
}
