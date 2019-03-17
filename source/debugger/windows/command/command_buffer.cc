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
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "command_buffer.hh"

using namespace jones::command_window;

void command_buffer::insert(int position, int input) {
  buffer_.insert(buffer_.begin() + position, input);
}

void command_buffer::erase(int position) {
  if (!buffer_.empty()) {
    buffer_.erase(buffer_.begin() + position);
  }
}

void command_buffer::clear() {
  buffer_.clear();
}

bool command_buffer::is_empty() const {
  std::string as_string = get();
  boost::trim(as_string);
  return buffer_.empty();
}

int command_buffer::size() const {
  return static_cast<int>(get().size());
}

std::string command_buffer::get() const {
  std::stringstream command;
  for (const auto &i : buffer_) {
    command << static_cast<char>(i);
  }
  return command.str();
}
