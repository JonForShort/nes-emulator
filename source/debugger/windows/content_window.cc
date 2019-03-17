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

#include "content_window.hh"
#include "log.hh"

using namespace jones;

namespace {

const char *const SCREEN_TITLE_FOCUSED = "[ *** content *** ]";
const char *const SCREEN_TITLE_UNFOCUSED = "[ content ]";

} // namespace

content_window::content_window(WINDOW *parent_window)
    : parent_window_(parent_window),
      window_(subwin(parent_window, 0, 0, 0, 0)) {}

content_window::~content_window() {
  delwin(window_);
}

void content_window::draw(int start_x, int start_y, int column_count, int line_count) {
  mvwin(window_, start_y, start_x);
  wresize(window_, line_count, column_count);
  box(window_, 0, 0);
  wrefresh(window_);
}

void content_window::on_focus() {
  mvwaddstr(window_, 0, 2, SCREEN_TITLE_FOCUSED);
}

void content_window::on_unfocus() {
  mvwaddstr(window_, 0, 2, SCREEN_TITLE_UNFOCUSED);
}

void content_window::on_key_pressed(int key) {
  LOG_TRACE << "on_key_pressed : "
            << "key [" << key << "]";
}

window_type content_window::type() {
  return window_type::CONTENT;
}