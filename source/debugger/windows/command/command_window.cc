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
#include <sstream>

#include "command_prompt.hh"
#include "command_window.hh"
#include "log.hh"

using namespace jones::command_window;

namespace {

const int SCREEN_CURSOR_X_POSITION = 2;

const int SCREEN_CURSOR_Y_POSITION = 2;

const int SCREEN_CURSOR_PADDING = 2;

const char *const SCREEN_TITLE_FOCUSED = "[ *** command *** ]";

const char *const SCREEN_TITLE_UNFOCUSED = "[ command ]";

} // namespace

command_window::command_window(WINDOW *parent_window)
    : parent_window_(parent_window),
      window_(subwin(parent_window, 0, 0, 0, 0)),
      command_prompt_() {
  command_prompt_.register_callback([this](prompt_event event, std::string command) {
    if (event == prompt_event::PROMPT_EVENT_COMPLETED) {
      process_command(command);
    }
  });
}

command_window::~command_window() {
  delwin(window_);
}

void command_window::draw(int start_x, int start_y, int column_count, int line_count) {
  mvwin(window_, start_y, start_x);
  wresize(window_, line_count, column_count);
  box(window_, 0, 0);
  wrefresh(window_);
}

void command_window::on_focus() {
  mvwaddstr(window_, 0, 2, SCREEN_TITLE_FOCUSED);
  noecho();
  reset_command_cursor(0);
}

void command_window::on_unfocus() {
  mvwaddstr(window_, 0, 2, SCREEN_TITLE_UNFOCUSED);
}

void command_window::on_key_pressed(int key) {
  command_prompt_.handle_input(key);
  update_command_prompt(command_prompt_.text(), command_prompt_.position());
}

window_type command_window::type() {
  return window_type::COMMAND;
}

void command_window::reset_command_cursor(int cursor_offset) const {
  int column_count = 0;
  int line_count = 0;
  getmaxyx(window_, line_count, column_count);

  const auto cursor_x_position = SCREEN_CURSOR_X_POSITION + SCREEN_CURSOR_PADDING + cursor_offset;
  wmove(parent_window_, line_count - SCREEN_CURSOR_Y_POSITION, cursor_x_position);
  mvwaddstr(window_, line_count - SCREEN_CURSOR_Y_POSITION, SCREEN_CURSOR_X_POSITION, "> ");
}

void command_window::update_command_prompt(const std::string &prompt_text, const int prompt_cursor_position) const {
  reset_command_cursor(0);
  clrtoeol();
  box(window_, 0, 0);
  waddstr(parent_window_, prompt_text.c_str());
  reset_command_cursor(prompt_cursor_position);
}

void command_window::process_command(const std::string &command) const {
  LOG_DEBUG << "process_command : handling command = " << command;
  if (command == "quit") {
    raise(SIGINT);
  }
  if (command == "history") {
    process_history_command();
  }
  if (command == "clear") {
    process_clear_command();
  }
}

void command_window::process_history_command() const {
}

void command_window::process_clear_command() const {
}
