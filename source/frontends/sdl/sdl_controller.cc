//
// MIT License
//
// Copyright 2019-2020
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
using controller_state = jones::controller::controller_state;

namespace {

button const button_mapping[] = {
    button::BUTTON_A,
    button::BUTTON_B,
    button::BUTTON_A,
    button::BUTTON_B,
    button::BUTTON_SELECT,
    button::BUTTON_START,
    button::BUTTON_SELECT,
    button::BUTTON_START,
};

auto const button_axis_offset = 100;

} // namespace

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
    controller_one_->set_controller_state(controller_state::CONTROLLER_STATE_CONNECTED);
  }
  if (SDL_NumJoysticks() > 1) {
    sdl_controller_two_ = SDL_GameControllerOpen(1);
    controller_two_->set_controller_state(controller_state::CONTROLLER_STATE_CONNECTED);
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
    controller_one_->set_controller_state(controller_state::CONTROLLER_STATE_DISCONNECTED);
  }
  if (sdl_controller_two_ != nullptr) {
    SDL_GameControllerClose(sdl_controller_two_);
    sdl_controller_two_ = nullptr;
    controller_two_->set_controller_state(controller_state::CONTROLLER_STATE_DISCONNECTED);
  }
  is_running_ = false;
}

auto sdl_controller::on_event(SDL_Event const event) -> void {
  switch (event.type) {
  case SDL_JOYAXISMOTION: {
    auto const controller = event.jbutton.which == 0 ? controller_one_ : controller_two_;
    auto const is_button_pushed = event.jaxis.value < (SDL_JOYSTICK_AXIS_MIN + button_axis_offset) || event.jaxis.value > (SDL_JOYSTICK_AXIS_MAX - button_axis_offset);
    auto const is_button_released = event.jaxis.value > -button_axis_offset && event.jaxis.value < button_axis_offset;
    if (is_button_pushed) {
      if (event.jaxis.axis == 0) {
        auto const button = event.jaxis.value > 0 ? button::BUTTON_RIGHT : button::BUTTON_LEFT;
        controller->set_button_state(button, button_state::BUTTON_STATE_DOWN);
      }
      if (event.jaxis.axis == 1) {
        auto const button = event.jaxis.value > 0 ? button::BUTTON_DOWN : button::BUTTON_UP;
        controller->set_button_state(button, button_state::BUTTON_STATE_DOWN);
      }
    }
    if (is_button_released) {
      for (auto const button : {button::BUTTON_UP, button::BUTTON_DOWN, button::BUTTON_LEFT, button::BUTTON_RIGHT}) {
        if (controller->get_button_state(button) == button_state::BUTTON_STATE_DOWN) {
          controller->set_button_state(button, button_state::BUTTON_STATE_UP);
        }
      }
    }
    break;
  }
  case SDL_JOYBUTTONUP:
  case SDL_JOYBUTTONDOWN: {
    auto const controller = event.jbutton.which == 0 ? controller_one_ : controller_two_;
    auto const button_state = event.jbutton.state == SDL_PRESSED ? button_state::BUTTON_STATE_DOWN : button_state::BUTTON_STATE_UP;
    auto const button = button_mapping[event.jbutton.button];
    controller->set_button_state(button, button_state);
    break;
  }
  case SDL_JOYDEVICEADDED: {
    if (event.jdevice.which == 0) {
      controller_one_->set_controller_state(controller::controller_state::CONTROLLER_STATE_CONNECTED);
    }
    if (event.jdevice.which == 1) {
      controller_one_->set_controller_state(controller::controller_state::CONTROLLER_STATE_DISCONNECTED);
    }
    break;
  }
  case SDL_JOYDEVICEREMOVED: {
    if (event.jdevice.which == 0) {
      controller_one_->set_controller_state(controller::controller_state::CONTROLLER_STATE_DISCONNECTED);
    }
    if (event.jdevice.which == 1) {
      controller_one_->set_controller_state(controller::controller_state::CONTROLLER_STATE_DISCONNECTED);
    }
    break;
  }
  default:
    break;
  }
}
