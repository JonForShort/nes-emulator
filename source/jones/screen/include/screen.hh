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
#ifndef JONES_SCREEN_SCREEN_HH
#define JONES_SCREEN_SCREEN_HH

namespace jones::screen {

constexpr auto screen_width = 256;

constexpr auto screen_height = 240;

struct pixel {

  uint8_t red;

  uint8_t blue;

  uint8_t green;
};

class screen {
public:
  virtual ~screen() = default;

  virtual auto set_pixel(uint16_t x_position, uint16_t y_position, pixel pixel) -> void = 0;

  virtual auto lock() -> void = 0;

  virtual auto unlock() -> void = 0;

  virtual auto render() -> void = 0;
};

} // namespace jones::screen

#endif // JONES_SCREEN_SCREEN_HH
