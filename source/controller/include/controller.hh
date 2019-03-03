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

namespace jones::controller {

enum class buttons {
  BUTTON_A,
  BUTTON_B,
  BUTTON_SELECT,
  BUTTON_START,
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_LEFT,
  BUTTON_RIGHT
};

class controller {
public:
  virtual ~controller() {}

  virtual auto on_connected() -> void = 0;
  virtual auto on_disconnected() -> void = 0;

  virtual auto is_button_pressed(buttons button) -> void = 0;
  virtual auto is_button_unpressed(buttons button) -> void = 0;
};

} // namespace jones::controller

#endif // JONES_CONTROLLER_CONTROLLER_HH
