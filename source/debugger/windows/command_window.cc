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
#include <boost/algorithm/string.hpp>
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
      command_buffer_(),
      command_history_(),
      command_offset_(-1) {}

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
  wrefresh(window_);
}

void command_window::on_focus() {
  noecho();
  reset_command_cursor();
}

void command_window::on_unfocus() {
}

void command_window::on_key_pressed(int key) {
  LOG_TRACE << "on_key_pressed : [before] "
            << "key [" << key << "] "
            << "offset [" << command_offset_ << "] "
            << "buffer [" << command_buffer_ << "]";

  switch (key) {
  case KEY_BACKSPACE:
    command_buffer_.pop_back();
    break;
  case KEY_ENTER:
  case KEY_ENTER_ALT:
    boost::trim(command_buffer_);
    if (!command_buffer_.empty()) {
      process_command();
      command_history_.push_back(command_buffer_);
      command_buffer_.clear();
      command_offset_ = command_history_.size();
    }
    break;
  case KEY_UP:
    if (command_offset_ > 0) {
      command_offset_--;
      command_buffer_ = command_history_[command_offset_];
    }
    break;
  case KEY_DOWN:
    if (command_offset_ >= 0 && (command_offset_ < static_cast<int>(command_history_.size()))) {
      command_offset_++;
      if (command_offset_ >= static_cast<int>(command_history_.size())) {
        command_buffer_.clear();
      } else {
        command_buffer_ = command_history_[command_offset_];
      }
    }
    break;
  default:
    command_buffer_.push_back(key);
    break;
  }

  update_command_prompt();

  LOG_TRACE << "on_key_pressed : [after] "
            << "key [" << key << "] "
            << "offset [" << command_offset_ << "] "
            << "buffer [" << command_buffer_ << "]";
}

window_type command_window::type() {
  return window_type::COMMAND;
}

void command_window::reset_command_cursor() const {
  int column_count = 0;
  int line_count = 0;
  getmaxyx(window_, line_count, column_count);
  wmove(parent_window_, line_count - 2, 4);
  mvwaddstr(window_, line_count - 2, 2, "> ");
}

void command_window::update_command_prompt() {
  reset_command_cursor();
  clrtoeol();
  box(window_, 0, 0);
  reset_command_cursor();
  for (auto &c : command_buffer_) {
    waddch(parent_window_, c);
  }
}

void command_window::process_command() {
  const std::string copied_command_buffer(command_buffer_);
  LOG_DEBUG << "process_command : handling command = " << copied_command_buffer;
  if (copied_command_buffer == "quit") {
    raise(SIGINT);
  }
}
