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
#ifndef JONES_DEBUGGER_LAYOUT_MANAGER_HH
#define JONES_DEBUGGER_LAYOUT_MANAGER_HH

#include <map>
#include <vector>

#include "window.hh"

namespace jones {

enum class layout_position {

  POSITION_TOP,

  POSITION_BOTTOM,
};

enum class layout_event {

};

class layout_manager final {

public:
  void register_window(windows::window_ptr registered_window, layout_position position);

  void update_layout(int height, int width) const;

  void rotate_window_focus(layout_position position);

  void rotate_position_focus();

  void handle_input(int key);

private:
  void update_focus() const;

private:
  using window_ptrs = std::vector<windows::window_ptr>;
  using layout_positions = std::vector<layout_position>;

  windows::window *focus_window_;

  layout_position focus_position_;

  window_ptrs windows_;

  layout_positions positions_;

  std::map<layout_position, std::vector<windows::window *>> window_layout_;
};

} // namespace jones

#endif // JONES_DEBUGGER_LAYOUT_MANAGER_HH