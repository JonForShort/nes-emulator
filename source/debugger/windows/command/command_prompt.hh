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
#ifndef JONES_DEBUGGER_WINDOWS_COMMAND_COMMAND_PROMPT_HH
#define JONES_DEBUGGER_WINDOWS_COMMAND_COMMAND_PROMPT_HH

#include <functional>
#include <vector>

#include "command_buffer.hh"

namespace jones::command_window {

enum class prompt_event {
  PROMPT_EVENT_COMPLETED,
};

typedef std::function<void(prompt_event, std::string)> event_callback;

class command_prompt {

public:
  int position() const;

  std::string text() const;

  void register_callback(event_callback callback);

  void handle_input(int input);

private:
  void notify_callbacks(prompt_event event, std::string parameter) const;

private:
  command_buffer current_buffer_;

  int current_buffers_offset_;

  int current_cursor_position_;

  std::vector<command_buffer> buffers_;

  std::vector<event_callback> callbacks_;
};

} // namespace jones::command_window

#endif // JONES_DEBUGGER_WINDOWS_COMMAND_COMMAND_PROMPT_HH
