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
#include <string>

#include "interface.hh"
#include "log.hh"
#include "windows/command_window.hh"

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

int register_signal_handler() {
  LOG_DEBUG << "registering signal handlers.";
  struct sigaction action;
  action.sa_handler = signal_handler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(SIGWINCH, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return -1;
  }
  if (sigaction(SIGINT, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return -1;
  }
  if (sigaction(SIGTERM, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return -1;
  }
  if (sigaction(SIGQUIT, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return -1;
  }
  if (sigaction(SIGCHLD, &action, NULL) < 0) {
    LOG_ERROR << "sigaction failed";
    return -1;
  }
  return 0;
}

int unregister_signal_handler() {
  LOG_DEBUG << "unregistering signal handlers.";
  signal(SIGWINCH, SIG_DFL);
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);
  signal(SIGQUIT, SIG_DFL);
  signal(SIGQUIT, SIG_DFL);
}

static const unsigned int command_window_height = 5;

} // namespace

interface::interface()
    : main_window_(initscr()), is_running_(false), screen_height_(0), screen_width_(0), focus_window_(window_type::COMMAND) {}

interface::~interface() {
  release();
}

void interface::initialize() {
  register_signal_handler();
}

void interface::release() {
  if (main_window_ != nullptr) {
    is_running_ = false;

    unregister_signal_handler();

    delwin(main_window_);
    main_window_ = nullptr;
    endwin();
  }
}

void interface::show() {
  update();
  command_window command_window(main_window_);
  noecho();
  is_running_ = true;
  while (is_running_) {
    command_window.draw(0, screen_height_ - command_window_height, screen_width_, screen_height_);
    command_window.on_focus();
    clrtoeol();
    wrefresh(main_window_);
    command_window.on_key_pressed(getch());
    is_running_ = false;
  }
}

void interface::update() {
  getmaxyx(main_window_, screen_height_, screen_width_);
}

bool interface::window_has_focus(window_type window) {
  return focus_window_ == window;
}

void interface::window_focus(window_type window) {
  focus_window_ = window;
  update();
}

window_type interface::window_focus() {
  return focus_window_;
}

void interface::rotate_window_focus() {
  switch (focus_window_) {
  case window_type::COMMAND:
    focus_window_ = window_type::CONTENT;
    update();
    break;
  case window_type::CONTENT:
    focus_window_ = window_type::COMMAND;
    update();
    break;
  }
}
