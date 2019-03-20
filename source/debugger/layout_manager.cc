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
#include "layout_manager.hh"
#include "log.hh"

using namespace jones;

void layout_manager::register_window(windows::window_ptr registered_window, layout_position position) {
  windows_.push_back(std::move(registered_window));
  window_layout_[position].push_back(registered_window.get());

  focus_window_ = windows_.front().get();
}

void layout_manager::update_layout(int height, int width) const {
  focus_window_->draw(0, 0, height, width);
  focus_window_->on_focus();
}

void layout_manager::update_focus() const {
}

void layout_manager::rotate_window_focus(layout_position position) {
  if (window_layout_.empty()) {
    LOG_WARNING << "unable to rotate; windows is empty";
    return;
  }
  if (window_layout_.size() == 1) {
    LOG_DEBUG << "only one window; no need to rotate";
    return;
  }
  auto &position_windows = window_layout_.at(position);
  std::rotate(position_windows.begin(), position_windows.begin() + 1, position_windows.end());
  update_focus();
}

void layout_manager::rotate_position_focus() {
  if (positions_.empty()) {
    LOG_WARNING << "unable to rotate; windows is empty";
    return;
  }
  if (positions_.size() == 1) {
    LOG_DEBUG << "only one window; no need to rotate";
    return;
  }
  std::rotate(positions_.begin(), positions_.begin() + 1, positions_.end());
  focus_position_ = *positions_.begin();
  update_focus();
}

void layout_manager::handle_input(int input) {
  focus_window_->on_key_pressed(input);
}