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
#include <curses.h>
#include <signal.h>

#include "command_window.hh"
#include "log.hh"

using namespace jones;

namespace {

static const unsigned int KEY_ENTER_ALT = 10;

} // namespace

command_window::command_window(WINDOW *parent_window)
    : parent_window_(parent_window),
      window_(subwin(parent_window, 0, 0, 0, 0)),
      input_buffer_() {}

command_window::~command_window() {
  release();
}

void command_window::release() {
  delwin(window_);
}

void command_window::draw(int start_x, int start_y, int column_count, int line_count) {
  mvwin(window_, start_y, start_x);
  wresize(window_, line_count, column_count);
  box(window_, 0, 0);
  mvwaddstr(window_, 0, 2, "[ command ]");
  mvwaddstr(window_, 2, 2, "> ");
  wrefresh(window_);
}

void command_window::on_focus() {
  noecho();
  wmove(parent_window_, 2, 4);
}

void command_window::on_unfocus() {
}

void command_window::on_key_pressed(char key) {
  LOG_DEBUG << "on_key_pressed : " << key;

  switch (key) {
  case KEY_BACKSPACE:
    input_buffer_.pop_back();
    break;
  case KEY_ENTER:
  case KEY_ENTER_ALT:
    process_command();
    input_buffer_.clear();
    break;
  default:
    input_buffer_.push_back(key);
    break;
  }
  update_input();
}

window_type command_window::type() {
  return window_type::COMMAND;
}

void command_window::reset_input_cursor() const {
  wmove(parent_window_, 2, 4);
}

void command_window::update_input() const {
  reset_input_cursor();
  clrtoeol();
  reset_input_cursor();
  for (auto &c : input_buffer_) {
    waddch(parent_window_, c);
  }
}

void command_window::process_command() const {
  const std::string command(input_buffer_.begin(), input_buffer_.end());
  LOG_DEBUG << "process_command : handling command = " << command;
  if (command == "quit") {
    raise(SIGINT);
  }
}
