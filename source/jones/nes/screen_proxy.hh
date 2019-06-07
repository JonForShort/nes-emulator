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
#ifndef JONES_NES_SCREEN_PROXY_HH
#define JONES_NES_SCREEN_PROXY_HH

#include "screen.hh"

namespace jones {

class screen_proxy : public screen::screen {
public:
  auto set_pixel(uint16_t x_position, uint16_t y_position, uint32_t pixel) -> void override {
    if (screen_) {
      screen_->set_pixel(x_position, y_position, pixel);
    }
  }

  auto render() -> void override {
    if (screen_) {
      screen_->render();
    }
  }

  auto attach_screen(std::unique_ptr<screen> screen) {
    screen_ = std::move(screen);
  }

private:
  std::unique_ptr<screen> screen_{};
};

} // namespace jones

#endif // JONES_NES_SCREEN_PROXY_HH
