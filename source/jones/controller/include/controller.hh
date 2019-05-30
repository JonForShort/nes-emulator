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
#ifndef JONES_CONTROLLER_CONTROLLER_HH
#define JONES_CONTROLLER_CONTROLLER_HH

#include "memory.hh"

namespace jones::controller {

enum class button {
  BUTTON_A,

  BUTTON_B,

  BUTTON_SELECT,

  BUTTON_START,

  BUTTON_UP,

  BUTTON_DOWN,

  BUTTON_LEFT,

  BUTTON_RIGHT,

  BUTTON_INVALID
};

enum class button_state {
  BUTTON_STATE_DOWN,

  BUTTON_STATE_UP,

  BUTTON_STATE_INVALID
};

enum class controller_state {
  CONNECTED,

  DISCONNECTED,

  INVALID
};

const char *button_to_string(button button);

const char *button_state_to_string(button_state button_state);

class controller final {
public:
  explicit controller(const memory &memory);

  ~controller();

  auto set_button_state(button button, button_state state) -> void;

  auto set_controller_state(controller_state state) -> void;

  auto read(uint16_t address) -> uint8_t;

  auto write(uint16_t address, uint8_t data) -> void;

private:
  class impl;

  std::unique_ptr<impl> impl_;
};

using controller_ptr = controller *;

} // namespace jones::controller

#endif // JONES_CONTROLLER_CONTROLLER_HH
