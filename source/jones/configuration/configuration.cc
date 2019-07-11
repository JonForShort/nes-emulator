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
#include <any>
#include <map>

#include "configuration.hh"

using namespace jones::configuration;

class configuration::impl {
public:
  template <typename T>
  auto set(property const property, T const &value) -> void {
    properties_[property] = std::any(value);
  }

  template <typename T>
  auto get(property const property, T const default_value) const -> T {
    const auto found_property = properties_.find(property);
    if (found_property == properties_.end()) {
      return default_value;
    }
    return std::any_cast<T>(found_property->second);
  }

private:
  std::map<property, std::any> properties_;
};

auto jones::configuration::get_mirror_mode(uint8_t const data) -> uint8_t {
  switch (data) {
  case static_cast<uint8_t>(mirror_mode::MIRROR_MODE_SINGLE_LOWER):
    return static_cast<uint8_t>(mirror_mode::MIRROR_MODE_SINGLE_LOWER);
  case static_cast<uint8_t>(mirror_mode::MIRROR_MODE_SINGLE_UPPER):
    return static_cast<uint8_t>(mirror_mode::MIRROR_MODE_SINGLE_UPPER);
  case static_cast<uint8_t>(mirror_mode::MIRROR_MODE_VERTICAL):
    return static_cast<uint8_t>(mirror_mode::MIRROR_MODE_VERTICAL);
  case static_cast<uint8_t>(mirror_mode::MIRROR_MODE_HORIZONTAL):
    return static_cast<uint8_t>(mirror_mode::MIRROR_MODE_HORIZONTAL);
  default:
    return static_cast<uint8_t>(mirror_mode::MIRROR_MODE_VERTICAL);
  }
}

template auto configuration::get<uint8_t>(property property, uint8_t default_value) const -> uint8_t;
template auto configuration::set<uint8_t>(property property, uint8_t const &value) -> void;

template auto configuration::get<bool>(property property, bool default_value) const -> bool;
template auto configuration::set<bool>(property property, bool const &value) -> void;

configuration::configuration() : pimpl_(new impl()) {}

auto configuration::instance() -> configuration & {
  static configuration instance;
  return instance;
}

template <typename T>
auto configuration::set(property const property, T const &value) -> void {
  return pimpl_->set<T>(property, value);
}

template <typename T>
auto configuration::get(property const property, T const default_value) const -> T {
  return pimpl_->get<T>(property, default_value);
}
