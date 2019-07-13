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
#include <string.h>

#include "log.hh"
#include "sdl_screen.hh"

using namespace jones::sdl;

namespace {

constexpr auto sdl_bit_depth = 32;

#if SDL_BYTEORDER == SDL_BIG_ENDIAN

constexpr auto sdl_red_mask = 0xFF000000;
constexpr auto sdl_green_mask = 0x00FF0000;
constexpr auto sdl_blue_mask = 0x0000FF00;
constexpr auto sdl_alpha_mask = 0x000000FF;

#else

constexpr auto sdl_red_mask = 0x000000FF;
constexpr auto sdl_green_mask = 0x0000FF00;
constexpr auto sdl_blue_mask = 0x00FF0000;
constexpr auto sdl_alpha_mask = 0xFF000000;

#endif

} // namespace

sdl_screen::sdl_screen(std::unique_ptr<sdl_screen_listener> listener)
    : listener_(std::move(listener)) {
}

sdl_screen::~sdl_screen() {
  uninitialize();
}

auto sdl_screen::initialize() -> void {
  show();
}

auto sdl_screen::uninitialize() -> void {
  hide();
}

auto sdl_screen::show() -> void {
  if (is_running_) {
    LOG_DEBUG << "screen is already being shown";
    return;
  }
  SDL_Init(SDL_INIT_HAPTIC);
  window_ = SDL_CreateWindow("Jones NES Emulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, jones::screen::screen_width, jones::screen::screen_height, SDL_WINDOW_RESIZABLE);
  if (window_ == nullptr) {
    LOG_ERROR << SDL_GetError();
    return;
  }
  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
  if (renderer_ == nullptr) {
    LOG_ERROR << SDL_GetError();
    return;
  }
  screen_ = SDL_CreateRGBSurface(0, jones::screen::screen_width, jones::screen::screen_height, sdl_bit_depth, sdl_red_mask, sdl_green_mask, sdl_blue_mask, sdl_alpha_mask);
  if (screen_ == nullptr) {
    LOG_ERROR << SDL_GetError();
    return;
  }
  texture_ = SDL_CreateTexture(renderer_, screen_->format->format, SDL_TEXTUREACCESS_STREAMING, jones::screen::screen_width, jones::screen::screen_height);
  if (texture_ == nullptr) {
    LOG_ERROR << SDL_GetError();
    return;
  }
  is_running_ = true;
}

auto sdl_screen::hide() -> void {
  if (!is_running_) {
    LOG_DEBUG << "screen is already being hidden";
    return;
  }
  if (texture_ != nullptr) {
    SDL_DestroyTexture(texture_);
    texture_ = nullptr;
  }
  if (renderer_ != nullptr) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_ != nullptr) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  if (screen_ != nullptr) {
    SDL_FreeSurface(screen_);
    screen_ = nullptr;
  }
  is_running_ = false;
}

auto sdl_screen::set_pixel(uint16_t const x_position, uint16_t const y_position, pixel const pixel) -> void {
  if (renderer_ == nullptr) {
    return;
  }
  auto const scaled_x_position = x_position * window_scale_;
  auto const scaled_y_position = y_position * window_scale_;

  auto const bytes_per_pixel = screen_->format->BytesPerPixel;
  auto const mapped_pixel = SDL_MapRGB(screen_->format, pixel.red, pixel.green, pixel.blue);

  auto *const pixel_one = static_cast<uint8_t *>(screen_->pixels) + (scaled_y_position * screen_->pitch) + (x_position * bytes_per_pixel);
  memcpy(pixel_one, &mapped_pixel, bytes_per_pixel);

  for (auto i = scaled_x_position; i < scaled_x_position + window_scale_; i++) {
    for (auto j = scaled_y_position; j < scaled_y_position + window_scale_; j++) {
      if ((i == scaled_x_position) && (j == scaled_y_position)) {
        continue;
      }
      auto *const pixel_two = static_cast<uint8_t *>(screen_->pixels) + j * screen_->pitch + i * bytes_per_pixel;
      memcpy(pixel_two, pixel_one, bytes_per_pixel);
    }
  }
}

auto sdl_screen::render() -> void {
  SDL_UpdateTexture(texture_, nullptr, screen_->pixels, screen_->pitch);
  SDL_RenderClear(renderer_);
  SDL_RenderCopy(renderer_, texture_, nullptr, nullptr);
  SDL_RenderPresent(renderer_);
}

auto sdl_screen::lock() -> void {
  if (SDL_MUSTLOCK(screen_)) {
    SDL_LockSurface(screen_);
  }
}

auto sdl_screen::unlock() -> void {
  if (SDL_MUSTLOCK(screen_)) {
    SDL_UnlockSurface(screen_);
  }
}

auto sdl_screen::on_event(SDL_Event const event) -> void {
  if (event.type == SDL_QUIT) {
    is_running_ = false;
    if (listener_ != nullptr) {
      listener_->on_screen_closed();
    }
  }
}
