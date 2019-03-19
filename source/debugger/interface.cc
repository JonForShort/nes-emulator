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

#include "interface.hh"
#include "log.hh"
#include "windows/command/command_window.hh"

using namespace jones;

namespace {

const unsigned int KEY_STAB_ALT = '\t';

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
      is_running_(true),
      screen_height_(0),
      screen_width_(0),
      focus_window_(nullptr),
      windows_() {}

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

    // todo: causes segmentation fault
    //    unregister_windows();

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
  windows_.push_back(std::make_unique<command_window::command_window>(interface_window_));
  focus_window_ = (*windows_.begin()).get();
}

void interface::unregister_windows() {
  LOG_DEBUG << "unregistering windows";
  windows_.clear();
}

void interface::show() {
  update();
  while (is_running_) {
    const auto &input = getch();
    switch (input) {
    case KEY_STAB:
    case KEY_STAB_ALT:
      rotate_window_focus();
      break;
    default:
      focus_window_->on_key_pressed(input);
      break;
    }
  }
}

void interface::update() {
  getmaxyx(interface_window_, screen_height_, screen_width_);
  focus_window_->draw(0, 0, screen_width_, screen_height_);
  focus_window_->on_focus();
}

bool interface::window_has_focus(window *focus_window) {
  return focus_window_ == focus_window;
}

void interface::window_focus(window *focus_window) {
  focus_window_ = focus_window;
  focus_window_->on_focus();
}

window *interface::window_focus() {
  return focus_window_;
}

void interface::rotate_window_focus() {
  if (windows_.empty()) {
    LOG_WARNING << "unable to rotate; windows is empty";
    return;
  }
  if (windows_.size() == 1) {
    LOG_DEBUG << "only one window; no need to rotate";
    return;
  }
  std::rotate(windows_.begin(), windows_.begin() + 1, windows_.end());
  const auto &next_window = *windows_.begin();
  window_focus(next_window.get());
}

void interface::on_window_change() {
}
