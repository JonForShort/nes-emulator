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
#ifndef JONES_SDL_SDL_CONTROLLER_HH
#define JONES_SDL_SDL_CONTROLLER_HH

#include "controller.hh"

#include <SDL2/SDL.h>

namespace jones::sdl {

class sdl_controller final {
public:
  sdl_controller(const controller::controller &controller_one, const controller::controller &controller_two)
      : controller_one_(controller_one), controller_two_(controller_two) {}

  ~sdl_controller() = default;

  void initialize();

  void uninitialize();

private:
  const controller::controller &controller_one_;

  const controller::controller &controller_two_;

  SDL_GameController *sdl_controller_one_ = nullptr;

  SDL_GameController *sdl_controller_two_ = nullptr;

  bool is_running_ = false;
};

} // namespace jones::sdl

#endif // JONES_SDL_SDL_CONTROLLER_HH
