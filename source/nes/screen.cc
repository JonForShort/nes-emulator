//
// MIT License
//
// Copyright 2017-2018
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
#include <iostream>
#include <SDL2/SDL.h>

#include "screen.hh"

using namespace jones;

Screen::Screen() : mIsRunning(false) {

}

Screen::~Screen() {
  mWindow = nullptr;
  mRenderer = nullptr;
  mIsRunning = false;
}

void Screen::initialize() {
  SDL_Init(SDL_INIT_HAPTIC); 
}

void Screen::show() {
  mWindow = SDL_CreateWindow("Jones NES Emulator",
					SDL_WINDOWPOS_UNDEFINED,
					SDL_WINDOWPOS_UNDEFINED,
					640, 480,
					SDL_WINDOW_OPENGL);
  if (mWindow == nullptr) {
    mIsRunning = false;
    return;
  }

  mRenderer = SDL_CreateRenderer(mWindow, -1, 0);
  if (mRenderer == nullptr) {
    mIsRunning = false;
    return;
  }

  fillWithColor(0, 0, 0, 0);

  mIsRunning = true;
  
  while (mIsRunning) {
    processEvent();
    renderScreen();
  }
}

void Screen::fillWithColor(u8 r, u8 g, u8 b, u8 a) {
  if (mRenderer != nullptr) {
    SDL_SetRenderDrawColor(mRenderer, r, g, b, a);
    SDL_RenderClear(mRenderer);
    SDL_RenderPresent(mRenderer);
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
