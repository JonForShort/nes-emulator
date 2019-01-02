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

main_screen::main_screen() : main_window_(initscr()) {}

main_screen::~main_screen() {
  release();
}

void main_screen::init() {
  
  signal(SIGINT, sigIntHandler);
  signal(SIGWINCH, sigWinchHandler);

  int lines;
  int cols;
  WINDOW* top;
  WINDOW* bottom;
  bool running = true;

  getmaxyx(main_window_, lines, cols);
  top = subwin(main_window_, lines - command_window_height, cols, 0, 0);
  bottom = subwin(main_window_, command_window_height, cols, lines - command_window_height, 0);

  noecho();
  
  while (running) {
    box(top, 0, 0);
    mvwaddstr(top, 0, 2, "[ debugger ]");

    box(bottom, 0, 0);
    mvwaddstr(bottom, 0, 2, "[ command ]");
    mvwaddstr(bottom, 2, 2, "> ");

    wmove(main_window_, lines - command_window_height + 2, 4);

    clrtoeol();
    wrefresh(main_window_);

    const auto line = get_line();
    if (line == "quit") {
      running = false;
    }
  }
}

void main_screen::release() {
  if (main_window_ != nullptr) {
    signal(SIGINT, SIG_DFL);
    signal(SIGWINCH, SIG_DFL);

    delwin(main_window_);
    main_window_ = nullptr;
    endwin();
  }
}

void main_screen::refresh() {

}
