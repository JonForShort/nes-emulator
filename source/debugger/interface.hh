//
// MIT License
//
// Copyright 2018
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
#ifndef JONES_DEBUGGER_INTERFACE_HH
#define JONES_DEBUGGER_INTERFACE_HH

#include <curses.h>
#include <memory.h>

namespace jones {

enum class window_type {
  COMMAND,
  CONTENT
};

class interface final {
public:
  interface();
  ~interface();

  void initialize();
  void release();

  void show();
  void update();

  void rotate_window_focus();
  bool window_has_focus(window_type window);
  void window_focus(window_type window);
  window_type window_focus();

private:
  WINDOW *main_window_;
  bool is_running_;

  unsigned int screen_height_;
  unsigned int screen_width_;

  window_type focus_window_;
};

} // namespace jones

#endif // JONES_DEBUGGER_INTERFACE_HH
