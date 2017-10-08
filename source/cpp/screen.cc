#include <SDL.h>

#include "screen.h"

using namespace jones;

void Screen::initialize() {
  SDL_Init(SDL_INIT_HAPTIC); 
}

void Screen::release() {
  SDL_Quit();
}
