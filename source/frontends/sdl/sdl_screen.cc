//
// MIT License
//
// Copyright 2019
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
#include <cstdint>

#include "log.hh"
#include "sdl_screen.hh"

using namespace jones::sdl;

sdl_screen::sdl_screen(std::unique_ptr<sdl_screen_listener> listener)
    : listener_(std::move(listener)) {
}

sdl_screen::~sdl_screen() {
  uninitialize();
}

void sdl_screen::initialize() {
  show();
}

void sdl_screen::uninitialize() {
  hide();
}

void sdl_screen::show() {
  if (is_running_) {
    LOG_DEBUG << "screen is already being shown";
    return;
  }
  SDL_Init(SDL_INIT_HAPTIC);
  window_ = SDL_CreateWindow("Jones NES Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, jones::screen::screen_width, jones::screen::screen_height, SDL_WINDOW_OPENGL);
  if (window_ == nullptr) {
    LOG_ERROR << "unable to create window";
    return;
  }
  renderer_ = SDL_CreateRenderer(window_, -1, 0);
  if (renderer_ == nullptr) {
    LOG_ERROR << "unable to create renderer";
    return;
  }
  fill_with_color(0, 0, 0, 0);
  is_running_ = true;
}

void sdl_screen::hide() {
  if (!is_running_) {
    LOG_DEBUG << "screen is already being hidden";
    return;
  }
  if (renderer_ != nullptr) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_ != nullptr) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  is_running_ = false;
}

void sdl_screen::set_pixel(const uint16_t x_position, const uint16_t y_position, const uint32_t pixel) {
  if (renderer_ != nullptr) {
    const uint8_t red = (pixel & 0xFF000000U) >> 24U;
    const uint8_t green = (pixel & 0xFF0000U) >> 16U;
    const uint8_t blue = (pixel & 0xFF00U) >> 8U;
    const uint8_t alpha = (pixel & 0xFFU);
    if (SDL_SetRenderDrawColor(renderer_, red, green, blue, alpha) == -1) {
      LOG_ERROR << SDL_GetError();
      return;
    }
    if (SDL_RenderDrawPoint(renderer_, x_position, y_position) == -1) {
      LOG_ERROR << SDL_GetError();
      return;
    }
  }
}

auto sdl_screen::render() -> void {
  SDL_RenderPresent(renderer_);
}

void sdl_screen::fill_with_color(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a) {
  if (renderer_ != nullptr) {
    if (SDL_SetRenderDrawColor(renderer_, r, g, b, a) == -1) {
      LOG_ERROR << SDL_GetError();
      return;
    }
    if (SDL_RenderClear(renderer_) == -1) {
      LOG_ERROR << SDL_GetError();
      return;
    }
  }
}

auto sdl_screen::on_event(const SDL_Event event) -> void {
  if (event.type == SDL_QUIT) {
    is_running_ = false;
    if (listener_ != nullptr) {
      listener_->on_screen_closed();
    }
  }
}
