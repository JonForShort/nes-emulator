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
#include "sdl_controller.hh"
#include "log.hh"

#include <SDL2/SDL.h>

using namespace jones::sdl;

auto sdl_controller::initialize() -> void {
  if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
    LOG_ERROR << "unable to initialize joysticks";
    return;
  }
  is_running_ = true;
  if (SDL_NumJoysticks() > 0) {
    sdl_controller_one_ = SDL_GameControllerOpen(0);
  }
  if (SDL_NumJoysticks() > 1) {
    sdl_controller_two_ = SDL_GameControllerOpen(1);
  }
}

auto sdl_controller::uninitialize() -> void {
  if (!is_running_) {
    LOG_DEBUG << "sdl controller is not running";
    return;
  }
  if (sdl_controller_one_ != nullptr) {
    SDL_GameControllerClose(sdl_controller_one_);
    sdl_controller_one_ = nullptr;
  }
  if (sdl_controller_two_ != nullptr) {
    SDL_GameControllerClose(sdl_controller_two_);
    sdl_controller_two_ = nullptr;
  }
  is_running_ = false;
}
