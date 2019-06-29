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
#ifndef JONES_CONFIGURATION_CONFIGURATION_HH
#define JONES_CONFIGURATION_CONFIGURATION_HH

#include <memory>

#include <cstdint>

namespace jones::configuration {

enum class property : uint8_t {
  PROPERTY_MIRROR_MODE = 0,

  PROPERTY_COUNT
};

class configuration {
public:
  configuration();

  static configuration &instance();

  template <typename T>
  auto set(property property, const T &value) -> void;

  template <typename T>
  auto get(property property, T default_value) const -> T;

private:
  class impl;

  std::unique_ptr<impl> pimpl_;
};

} // namespace jones::configuration

#endif // JONES_CONFIGURATION_CONFIGURATION_HH
