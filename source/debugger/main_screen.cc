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
#include <signal.h>
#include <string>

#include "main_screen.hh"
#include "windows/command_window.hh"

using namespace jones;

namespace {

  static const unsigned int command_window_height = 5;

  std::string get_line() {
    std::string result;
    while(true) {
      const auto ch = getch();
      switch (ch) {
      case '\n':
      case ERR:
	return result;
      default:
	result += ch;
	addch(ch);
	break;
      }
    }
  }

  void sigIntHandler(int sig_num) {

  }

  void sigWinchHandler(int sig_num) {
    endwin();
    refresh();
    clear();
  }
}

main_screen::main_screen():
  main_window_(initscr()),
  is_running_(false),
  screen_height_(0),
  screen_width_(0) {
}

main_screen::~main_screen() {
  release();
}

void main_screen::initialize() {
  signal(SIGINT, sigIntHandler);
  signal(SIGWINCH, sigWinchHandler);
}

void main_screen::release() {
  if (main_window_ != nullptr) {
    is_running_ = false;

    signal(SIGINT, SIG_DFL);
    signal(SIGWINCH, SIG_DFL);

    delwin(main_window_);
    main_window_ = nullptr;
    endwin();
  }
}

void main_screen::show() {
  update();
  command_window command_window(main_window_, screen_height_, screen_width_);
  noecho();
  is_running_ = true;
  while (is_running_) {
    command_window.redraw(screen_height_, screen_width_);
    clrtoeol();
    wrefresh(main_window_);
    const auto line = get_line();
    if (line == "quit") {
      is_running_ = false;
    }
  }
}

void main_screen::update() {
  getmaxyx(main_window_, screen_height_, screen_width_);
}
