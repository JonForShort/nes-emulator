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
#include <algorithm>
#include <curses.h>
#include <signal.h>
#include <string>

#include "input.hh"
#include "interface.hh"
#include "layout_manager.hh"
#include "log.hh"
#include "windows/command/command_window.hh"
#include "windows/content/content_window.hh"

using namespace jones;
using namespace jones::windows;

namespace {

void initialize_curses() {
  cbreak();
  keypad(stdscr, TRUE);
  noecho();
}

} // namespace

interface &interface::instance() {
  static interface instance;
  return instance;
}

interface::interface()
    : interface_window_(initscr()),
      layout_manager_(),
      is_running_(true),
      screen_height_(0),
      screen_width_(0) {}

interface::~interface() {
  release();
}

void interface::initialize() {
  initialize_curses();
  register_signal_handlers();
  register_windows();
}

void interface::release() {
  if (interface_window_ != nullptr) {
    is_running_ = false;
    ungetch(0);

    unregister_signal_handlers();
    unregister_windows();

    delwin(interface_window_);
    interface_window_ = nullptr;
    endwin();
  }
}

void interface::handle_signal(int signal_number) {
  LOG_DEBUG << "signal handled = " << signal_number;
  switch (signal_number) {
  case SIGWINCH:
    instance().on_window_change();
    break;
  case SIGINT:
    instance().release();
    break;
  default:
    break;
  }
}

void interface::register_signal_handlers() {
  LOG_DEBUG << "registering signal handlers.";
  struct sigaction action {};
  action.sa_handler = interface::handle_signal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(SIGWINCH, &action, nullptr) < 0) {
    LOG_ERROR << "sigaction failed";
    return;
  }
  if (sigaction(SIGINT, &action, nullptr) < 0) {
    LOG_ERROR << "sigint failed";
    return;
  }
  if (sigaction(SIGTERM, &action, nullptr) < 0) {
    LOG_ERROR << "sigterm failed";
    return;
  }
  if (sigaction(SIGQUIT, &action, nullptr) < 0) {
    LOG_ERROR << "sigquit failed";
    return;
  }
  if (sigaction(SIGCHLD, &action, nullptr) < 0) {
    LOG_ERROR << "sigchld failed";
    return;
  }
}

void interface::unregister_signal_handlers() {
  LOG_DEBUG << "unregistering signal handlers";
  signal(SIGWINCH, SIG_DFL);
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);
  signal(SIGQUIT, SIG_DFL);
}

void interface::register_windows() {
  LOG_DEBUG << "registering windows";

  window_ptr command_window = std::make_unique<windows::command_window>(interface_window_);
  layout_manager_.register_window(std::move(command_window), layout_position ::POSITION_BOTTOM);
}

void interface::unregister_windows() {
  LOG_DEBUG << "unregistering windows";
}

void interface::show() {
  update();
  while (is_running_) {
    const auto &input = getch();
    if (input::is_tab(input)) {
      layout_manager_.rotate_position_focus();
    } else if (input::is_control_plus_tab(input)) {
      layout_manager_.rotate_window_focus();
    } else {
      layout_manager_.handle_input(input);
    }
  }
}

void interface::update() {
  getmaxyx(interface_window_, screen_height_, screen_width_);
  layout_manager_.update_layout(screen_height_, screen_width_);
}

void interface::on_window_change() {
  update();
}