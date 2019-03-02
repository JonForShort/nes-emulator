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
#include "windows/command_window.hh"
#include "windows/window.hh"

using namespace jones;

namespace {

void signal_handler(int signo) {
  LOG_DEBUG << "signal handled = " << signo;
  if (signo == SIGWINCH) {
    endwin();
    refresh();
    clear();
  }
}

static const unsigned int command_window_height = 5;

} // namespace

interface::interface()
    : interface_window_(initscr()),
      is_running_(false),
      screen_height_(0),
      screen_width_(0),
      focus_window_(nullptr),
      windows_() {}

interface::~interface() {
  release();
}

void interface::initialize() {
  register_signal_handlers();
  register_windows();
}

void interface::release() {
  if (interface_window_ != nullptr) {
    is_running_ = false;

    unregister_signal_handlers();
    unregister_windows();

    delwin(interface_window_);
    interface_window_ = nullptr;
    endwin();
  }
}

void interface::register_signal_handlers() {
  LOG_DEBUG << "registering signal handlers.";
  struct sigaction action;
  action.sa_handler = signal_handler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(SIGWINCH, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return;
  }
  if (sigaction(SIGINT, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return;
  }
  if (sigaction(SIGTERM, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return;
  }
  if (sigaction(SIGQUIT, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return;
  }
  if (sigaction(SIGCHLD, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return;
  }
}

void interface::unregister_signal_handlers() {
  LOG_DEBUG << "unregistering signal handlers";
  signal(SIGWINCH, SIG_DFL);
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);
  signal(SIGQUIT, SIG_DFL);
  signal(SIGQUIT, SIG_DFL);
}

void interface::register_windows() {
  LOG_DEBUG << "registering windows";
  windows_.push_back(std::make_unique<command_window>(interface_window_));
  focus_window_ = (*windows_.begin()).get();
}

void interface::unregister_windows() {
  LOG_DEBUG << "unregistering windows";
}

void interface::show() {
  update();
  noecho();
  is_running_ = true;
  while (is_running_) {
    focus_window_->draw(0, screen_height_ - command_window_height, screen_width_, screen_height_);
    clrtoeol();
    wrefresh(interface_window_);
    const auto &input = getch();
    focus_window_->on_key_pressed(input);
    if (input == 't') {
      rotate_window_focus();
    } else {
      is_running_ = false;
    }
  }
}

void interface::update() {
  getmaxyx(interface_window_, screen_height_, screen_width_);
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
  std::rotate(windows_.begin(), windows_.begin() + 1, windows_.end());
  const auto &next_window = *windows_.begin();
  window_focus(next_window.get());
}
