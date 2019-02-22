//
// MIT License
//
// Copyright 2019
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
#ifndef JONES_DEBUGGER_LOG_HH
#define JONES_DEBUGGER_LOG_HH

namespace jones {

enum class log_level {
  VERBOSE,
  DEBUG,
  WARNING,
  ERROR
};

enum class log_sink {
  NONE,
  CONSOLE,
  FILE
};

class log {
public:
  log();
  ~log();

  void set_minimum_level(log_level level);
  void set_log_sink(log_sink sink);

private:
  void update_log();

  log_sink sink_ = log_sink::NONE;
  log_level level_ = log_level::WARNING;
};

}; // namespace jones

#endif // JONES_DEBUGGER_LOG_HH
