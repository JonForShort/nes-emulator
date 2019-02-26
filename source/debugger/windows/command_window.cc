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

using namespace jones;

namespace {
static const unsigned int command_window_height = 5;
}

command_window::command_window(WINDOW *parent_window, int line_count, int column_count)
    : window_(subwin(parent_window, command_window_height, column_count, line_count - command_window_height, 0)) {}

command_window::~command_window() {}

void command_window::draw(int line_count, int column_count) {
  box(window_, 0, 0);

  mvwaddstr(window_, 0, 2, "[ command ]");
  mvwaddstr(window_, 2, 2, "> ");

  wrefresh(window_);

  clrtoeol();
}

void command_window::on_focus() {
}

void command_window::on_unfocus() {
}
