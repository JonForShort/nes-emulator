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
#ifndef JONES_NES_NES_CONTROLLER_HH
#define JONES_NES_NES_CONTROLLER_HH

#include "controller.hh"
#include "memory.hh"

#include <boost/core/ignore_unused.hpp>

namespace jones {

class nes_controller : public controller::controller {
public:
  explicit nes_controller(const memory &memory) : memory_(memory) {
    boost::ignore_unused(memory_);
  }

  ~nes_controller() override = default;

  auto set_button_state(const jones::controller::button_state state) -> void override {
    boost::ignore_unused(state);
  }

  auto set_controller_state(const jones::controller::controller_state state) -> void override {
    boost::ignore_unused(state);
  }

private:
  const memory &memory_;
};

class mapped_nes_controller {
public:
  explicit mapped_nes_controller(controller::controller *controller) : controller_(controller) {
    boost::ignore_unused(controller_);
  }

  auto read(const uint16_t address) const -> uint8_t {
    boost::ignore_unused(address);
    return 0;
  }

  auto write(const uint16_t address, const uint8_t data) -> void {
    boost::ignore_unused(address, data);
  }

private:
  controller::controller *controller_;
};

} // namespace jones

#endif // JONES_NES_NES_CONTROLLER_HH
