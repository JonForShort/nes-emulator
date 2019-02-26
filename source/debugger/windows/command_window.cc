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
#include <curses.h>

#include "command_window.hh"
#include "log.hh"

using namespace jones;

namespace {

static const unsigned int command_window_height = 5;

} // namespace

command_window::command_window(WINDOW *parent_window)
    : parent_window_(parent_window), window_(nullptr) {}

command_window::~command_window() {
  release();
}

void command_window::release() {
  if (window_ != nullptr) {
    delwin(window_);
    window_ = nullptr;
  }
}

void command_window::draw(int start_x, int start_y, int column_count, int line_count) {
  release();

  window_ = subwin(parent_window_, line_count, column_count, start_y, start_x);

  box(window_, 0, 0);

  mvwaddstr(window_, 0, 2, "[ command ]");
  mvwaddstr(window_, 2, 2, "> ");

  wrefresh(window_);

  clrtoeol();
}

void command_window::on_focus() {
  if (window_ != nullptr) {
    wmove(window_, 2, 2);
  }
}

void command_window::on_unfocus() {
}

void command_window::on_key_pressed(char key) {
  LOG_DEBUG << "on_key_pressed : " << key;
}
