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
#include <curses.h>

#include "base_window.hh"
#include "log.hh"

using namespace jones::windows;

base_window::base_window(WINDOW *parent_window)
    : parent_window_(parent_window),
      window_(subwin(parent_window, 0, 0, 0, 0)),
      start_y_(0),
      start_x_(0) {
}

base_window::~base_window() {
  if (window_ != nullptr) {
    delwin(window_);
    window_ = nullptr;
  }
}

void base_window::on_focus() {
  if (mvwaddstr(window_, 0, 2, title()) == ERR) {
    LOG_ERROR << "failed to put title on window";
    return;
  }
}

void base_window::on_unfocus() {
  if (mvwaddstr(window_, 0, 2, title()) == ERR) {
    LOG_ERROR << "failed to put title on window";
    return;
  }
}

void base_window::set_last_drawn(int start_y, int start_x, int line_count, int column_count) {
  start_y_ = start_y;
  start_x_ = start_x;
  line_count_ = line_count;
  column_count_ = column_count;
}

void base_window::draw(int start_y, int start_x, int line_count, int column_count) {
  if (wclear(window_) == ERR) {
    LOG_ERROR << "failed to clear window";
    return;
  }
  if (wresize(window_, line_count, column_count) == ERR) {
    LOG_ERROR << "failed to resize window";
    return;
  }
  if (mvwin(window_, start_y, start_x) == ERR) {
    LOG_ERROR << "failed to move window";
    return;
  }
  if (box(window_, 0, 0) == ERR) {
    LOG_ERROR << "failed to draw box around window";
    return;
  }
  if (mvwaddstr(window_, 0, 2, title()) == ERR) {
    LOG_ERROR << "failed to put title on window";
    return;
  }
  for (int i = 0; i < line_count && i < static_cast<int>(window_buffer_.size()); i++) {
    mvwaddstr(window_, i, 0, window_buffer_[i].c_str());
  }

  set_last_drawn(start_y, start_x, line_count, column_count);
  on_drawn();

  if (wrefresh(window_) == ERR) {
    LOG_ERROR << "failed to refresh window";
    return;
  }
}

WINDOW *base_window::window() const {
  return window_;
}

int base_window::line_count() const {
  return line_count_;
}

int base_window::column_count() const {
  return column_count_;
}

void base_window::append_window_buffer(const std::string &buffer) {
  window_buffer_.push_back(buffer);
}
