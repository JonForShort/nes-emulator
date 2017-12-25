#include <iostream>
#include <SDL.h>

#include "screen.h"

using namespace jones;

Screen::Screen() : mIsRunning(false) {

}

void Screen::initialize() {
  SDL_Init(SDL_INIT_HAPTIC); 
}

void Screen::showMain() {
  SDL_Window *window = SDL_CreateWindow("Jones NES Emulator",
					SDL_WINDOWPOS_UNDEFINED,
					SDL_WINDOWPOS_UNDEFINED,
					640, 480,
					SDL_WINDOW_OPENGL);
  if (window != nullptr) {
    std::cout << "successfully created window." << std::endl;
    mIsRunning = true;
  }
  while (mIsRunning) {
    processEvent();
    renderScreen();
  }
}

void Screen::processEvent() {
  while (SDL_PollEvent(&mEvents)) {
    if(mEvents.type == SDL_QUIT)
      mIsRunning = false;
  }
}

void Screen::renderScreen() {

}

void Screen::release() {
  SDL_Quit();
}
