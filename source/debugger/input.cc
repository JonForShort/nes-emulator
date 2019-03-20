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
#include <input.hh>

using namespace jones;

namespace {

const int KEY_ENTER_ALT = 10;

const int KEY_BACKSPACE_ALT = 127;

const int KEY_STAB_ALT = '\t';

} // namespace

bool input::is_enter(int key) {
  switch (key) {
  case KEY_ENTER:
  case KEY_ENTER_ALT:
    return true;
  default:
    return false;
  }
}

bool input::is_backspace(int key) {
  switch (key) {
  case KEY_BACKSPACE:
  case KEY_BACKSPACE_ALT:
    return true;
  default:
    return false;
  }
}

bool input::is_key_up(int key) {
  return KEY_UP == key;
}

bool input::is_key_down(int key) {
  return KEY_DOWN == key;
}

bool input::is_key_left(int key) {
  return KEY_LEFT == key;
}

bool input::is_key_right(int key) {
  return KEY_RIGHT == key;
}

bool input::is_control_plus(char c, int key) {
  return (c & 0x1f) == key;
}

bool input::is_tab(int key) {
  switch (key) {
  case KEY_STAB:
  case KEY_STAB_ALT:
    return true;
  default:
    return false;
  }
}
