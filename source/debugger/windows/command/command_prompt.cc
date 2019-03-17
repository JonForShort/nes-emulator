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
#include <curses.h>

#include "command_prompt.hh"

using namespace jones::command_window;

namespace {

const int KEY_ENTER_ALT = 10;
const int KEY_BACKSPACE_ALT = 127;

} // namespace

#define KEY_CONTROL_PLUS(key) ((key)&0x1f)

void command_prompt::handle_input(int key) {
  switch (key) {
  case KEY_BACKSPACE:
  case KEY_BACKSPACE_ALT:
    current_buffer_.erase(current_cursor_position_ - 1);
    current_cursor_position_ = current_buffer_.size();
    break;
  case KEY_ENTER:
  case KEY_ENTER_ALT:
    if (!current_buffer_.is_empty()) {
      notify_callbacks(prompt_event::PROMPT_EVENT_COMPLETED, current_buffer_.get());
      buffers_.push_back(current_buffer_);
      current_buffer_.clear();
      current_buffers_offset_ = static_cast<int>(buffers_.size());
    }
    current_cursor_position_ = current_buffer_.size();
    break;
  case KEY_UP:
    if (current_buffers_offset_ > 0) {
      current_buffers_offset_--;
      current_buffer_ = buffers_[current_buffers_offset_];
    }
    current_cursor_position_ = current_buffer_.size();
    break;
  case KEY_DOWN:
    if (current_buffers_offset_ >= 0 && (current_buffers_offset_ < static_cast<int>(buffers_.size()))) {
      current_buffers_offset_++;
      if (current_buffers_offset_ >= static_cast<int>(buffers_.size())) {
        current_buffer_.clear();
      } else {
        current_buffer_ = buffers_[current_buffers_offset_];
      }
      current_cursor_position_ = current_buffer_.size();
    }
    break;
  case KEY_LEFT:
    if (current_cursor_position_ > 0) {
      current_cursor_position_--;
    }
    break;
  case KEY_RIGHT:
    if (current_cursor_position_ < current_buffer_.size()) {
      current_cursor_position_++;
    }
    break;
  case KEY_CONTROL_PLUS('a'):
    current_cursor_position_ = 0;
    break;
  case KEY_CONTROL_PLUS('e'):
    current_cursor_position_ = current_buffer_.size();
    break;
  default:
    current_buffer_.insert(current_cursor_position_, key);
    current_cursor_position_++;
    break;
  }
}

#undef KEY_CONTROL_PLUS

int command_prompt::position() const {
  return current_cursor_position_;
}

std::string command_prompt::text() const {
  return current_buffer_.get();
}

void command_prompt::notify_callbacks(prompt_event event, std::string parameter) const {
  for (auto &callback : callbacks_) {
    callback(event, parameter);
  }
}

void command_prompt::register_callback(event_callback callback) {
  callbacks_.push_back(callback);
}
