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
#ifndef JONES_SDL_SDL_MANAGER_HH
#define JONES_SDL_SDL_MANAGER_HH

#include "sdl_component.hh"

#include <SDL2/SDL.h>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace jones::sdl {

class sdl_manager final {
public:
  sdl_manager() = default;

  ~sdl_manager();

  auto initialize() -> void;

  auto uninitialize() -> void;

  auto add_component(sdl_component *listener) -> void;

  auto remove_component(sdl_component *listener) -> void;

private:
  auto process_events() -> void;

private:
  std::vector<sdl_component *> components_{};

  std::atomic<bool> is_running_{};

  std::unique_ptr<std::thread> running_thread_{};
};

} // namespace jones::sdl

#endif // JONES_SDL_SDL_MANAGER_HH
