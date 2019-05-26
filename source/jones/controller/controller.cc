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
#include "controller.hh"

#include <boost/core/ignore_unused.hpp>

using namespace jones::controller;

class controller::impl {
public:
  explicit impl(const memory &memory) : memory_(memory) {
    boost::ignore_unused(memory_);
  }

  ~impl() = default;

  auto set_button_state(const button_state state) -> void {
    boost::ignore_unused(state);
  }

  auto set_controller_state(const controller_state state) -> void {
    boost::ignore_unused(state);
  }

  auto read(const uint16_t address) -> uint8_t {
    boost::ignore_unused(address);
    return 0;
  }

  auto write(const uint16_t address, const uint8_t data) -> void {
    boost::ignore_unused(address, data);
  }

private:
  const memory &memory_;
};

controller::controller(const memory &memory)
    : impl_(std::make_unique<impl>(memory)) {
}

controller::~controller() = default;

auto controller::set_button_state(const button_state state) -> void {
  impl_->set_button_state(state);
}

auto controller::set_controller_state(const controller_state state) -> void {
  impl_->set_controller_state(state);
}

auto controller::read(const uint16_t address) -> uint8_t {
  return impl_->read(address);
}

auto controller::write(const uint16_t address, const uint8_t data) -> void {
  impl_->write(address, data);
}
