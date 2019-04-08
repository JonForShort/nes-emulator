//
// MIT License
//
// Copyright 2017-2019
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
#include <SDL2/SDL.h>
#include <iostream>

#include "screen.hh"

using namespace jones;

screen::screen() : events_(), window_(nullptr), renderer_(nullptr), is_running_(false) {}

screen::~screen() {
  window_ = nullptr;
  renderer_ = nullptr;
  is_running_ = false;
}

void screen::initialize() { SDL_Init(SDL_INIT_HAPTIC); }

void screen::show() {
  window_ = SDL_CreateWindow("Jones NES Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, SDL_WINDOW_OPENGL);
  if (window_ == nullptr) {
    is_running_ = false;
    return;
  }
  renderer_ = SDL_CreateRenderer(window_, -1, 0);
  if (renderer_ == nullptr) {
    is_running_ = false;
    return;
  }
  fill_with_color(0, 0, 0, 0);

  is_running_ = true;
  while (is_running_) {
    process_events();
    render_screen();
  }
}

void screen::fill_with_color(u8 r, u8 g, u8 b, u8 a) {
  if (renderer_ != nullptr) {
    SDL_SetRenderDrawColor(renderer_, r, g, b, a);
    SDL_RenderClear(renderer_);
    SDL_RenderPresent(renderer_);
  }
}

void screen::process_events() {
  while (SDL_PollEvent(&events_)) {
    if (events_.type == SDL_QUIT)
      is_running_ = false;
  }
}

void screen::render_screen() {}

void screen::release() { SDL_Quit(); }
