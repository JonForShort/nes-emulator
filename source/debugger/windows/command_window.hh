//
// MIT License
//
// Copyright 2018-2019
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
#ifndef JONES_DEBUGGER_WINDOWS_COMMAND_WINDOW_HH
#define JONES_DEBUGGER_WINDOWS_COMMAND_WINDOW_HH

#include <curses.h>
#include <string>
#include <vector>

#include "window.hh"

namespace jones {

class command_window final : public window {
public:
  explicit command_window(WINDOW *parent_window);
  ~command_window() override;
  window_type type() override;
  void on_focus() override;
  void on_unfocus() override;
  void on_key_pressed(int key) override;
  void draw(int start_x, int start_y, int column_count, int line_count) override;

private:
  void reset_command_cursor() const;
  void update_command_prompt();

  void process_command();
  void process_history_command();
  void process_clear_command();

private:
  WINDOW *parent_window_;
  WINDOW *window_;
  std::vector<int> command_buffer_;
  std::vector<std::vector<int>> command_history_;
  int command_offset_;
};

} // namespace jones

#endif // JONES_DEBUGGER_WINDOWS_COMMAND_WINDOW_HH
