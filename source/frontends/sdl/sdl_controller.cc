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

using button = jones::controller::button;
using button_state = jones::controller::button_state;

namespace {

const button button_mapping[] = {
    button::BUTTON_A,
    button::BUTTON_B,
    button::BUTTON_A,
    button::BUTTON_B,
    button::BUTTON_SELECT,
    button::BUTTON_START,
    button::BUTTON_SELECT,
    button::BUTTON_START,
};

}

auto sdl_controller::initialize() -> void {
  if (is_running_) {
    LOG_DEBUG << "sdl controller is already running";
    return;
  }
  is_running_ = true;
  if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
    LOG_ERROR << "unable to initialize joysticks";
    return;
  }
  SDL_JoystickEventState(SDL_ENABLE);
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

auto sdl_controller::on_event(const SDL_Event event) -> void {
  switch (event.type) {
  case SDL_JOYAXISMOTION: {
    const auto controller = event.jbutton.which == 0 ? controller_one_ : controller_two_;
    const auto button_state = event.jbutton.state == SDL_PRESSED ? button_state::BUTTON_STATE_DOWN : button_state::BUTTON_STATE_UP;
    if ((event.jaxis.value < (SDL_JOYSTICK_AXIS_MIN + 100)) || (event.jaxis.value > (SDL_JOYSTICK_AXIS_MAX - 100))) {
      auto button = button::BUTTON_INVALID;
      if (event.jaxis.axis == 0) {
        button = event.jaxis.value > 0 ? button::BUTTON_RIGHT : button::BUTTON_LEFT;
        controller->set_button_state(button, button_state);
      }
      if (event.jaxis.axis == 1) {
        button = event.jaxis.value > 0 ? button::BUTTON_DOWN : button::BUTTON_UP;
        controller->set_button_state(button, button_state);
      }
      LOG_DEBUG << "sdl_controller::on_event : "
                << "button [" << controller::button_to_string(button) << "] "
                << "button_state [" << controller::button_state_to_string(button_state) << "]";
    }
    break;
  }
  case SDL_JOYBUTTONUP:
  case SDL_JOYBUTTONDOWN: {
    const auto controller = event.jbutton.which == 0 ? controller_one_ : controller_two_;
    const auto button_state = event.jbutton.state == SDL_PRESSED ? button_state::BUTTON_STATE_DOWN : button_state::BUTTON_STATE_UP;
    const auto button = button_mapping[event.jbutton.button];
    controller->set_button_state(button, button_state);
    LOG_DEBUG << "sdl_controller::on_event : "
              << "button [" << controller::button_to_string(button) << "] "
              << "button_state [" << controller::button_state_to_string(button_state) << "]";
    break;
  }
  default:
    break;
  }
}