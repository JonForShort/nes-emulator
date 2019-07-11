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
#ifndef JONES_SDL_SDL_SCREEN_HH
#define JONES_SDL_SDL_SCREEN_HH

#include "screen.hh"
#include "sdl_component.hh"

#include <SDL2/SDL.h>
#include <atomic>
#include <thread>

namespace jones::sdl {

class sdl_screen_listener {
public:
  virtual ~sdl_screen_listener() = default;

  virtual auto on_screen_closed() -> void = 0;
};

class sdl_screen : public screen::screen, public sdl_component {
public:
  explicit sdl_screen(std::unique_ptr<sdl_screen_listener> listener = nullptr);

  ~sdl_screen() override;

  auto initialize() -> void override;

  auto uninitialize() -> void override;

  auto set_pixel(uint16_t x_position, uint16_t y_position, uint32_t pixel) -> void override;

  auto render() -> void override;

  auto on_event(SDL_Event event) -> void override;

private:
  auto show() -> void;

  auto hide() -> void;

  auto fill_with_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) -> void;

private:
  SDL_Window *window_ = nullptr;

  SDL_Renderer *renderer_ = nullptr;

  std::atomic<bool> is_running_ = false;

  std::unique_ptr<sdl_screen_listener> listener_ = nullptr;
};

} // namespace jones::sdl

#endif // JONES_SDL_SDL_SCREEN_HH
