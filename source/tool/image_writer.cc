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
#include <boost/core/ignore_unused.hpp>
#include <memory>

#include "boost_gil.h"
#include "image_color.hh"
#include "image_writer.hh"

using namespace jones::tool;

namespace bg = boost::gil;

class image_writer::image_writer_impl {
public:
  void write(const char *file_path) {
    boost::ignore_unused(file_path);
    if (!image_) {
      return;
    }
    auto view = bg::view(*image_);
    bg::write_view(file_path, view, bg::image_write_info<bg::png_tag>());
  }

  void size(int height, int width) {
    boost::ignore_unused(height);
    boost::ignore_unused(width);
    image_ = std::make_unique<bg::rgb8_image_t>(height, width);
  }

  void set_pixel(COLOR color, int x_postion, int y_position) {
    boost::ignore_unused(color);
    if (!image_) {
      return;
    }
    uint8_t color_encoded[3] = {0};
    switch (color) {
    case COLOR::WHITE:
      color_encoded[0] = 0xFF;
      color_encoded[1] = 0xFF;
      color_encoded[2] = 0XFF;
      break;
    case COLOR::BLACK: {
      color_encoded[0] = 0x00;
      color_encoded[1] = 0x00;
      color_encoded[2] = 0x00;
      break;
    }
    case COLOR::LIGHT_GRAY:
      color_encoded[0] = 0xC0;
      color_encoded[1] = 0xC0;
      color_encoded[2] = 0xC0;
      break;
    case COLOR::DARK_GRAY:
      color_encoded[0] = 0x69;
      color_encoded[1] = 0x69;
      color_encoded[2] = 0x69;
      break;
    default:
      break;
    }
    auto view = bg::view(*image_);
    view(x_postion, y_position) = bg::rgb8_pixel_t(
        color_encoded[0], color_encoded[1], color_encoded[2]);
  }

private:
  std::unique_ptr<bg::rgb8_image_t> image_;
};

image_writer::image_writer() : pimpl_{std::make_unique<image_writer_impl>()} {
}

image_writer::~image_writer() = default;

image_writer::image_writer(image_writer &&) noexcept = default;

image_writer &image_writer::operator=(image_writer &&) noexcept = default;

void image_writer::write(const char *file_path) {
  pimpl_->write(file_path);
}

void image_writer::size(int height, int width) {
  pimpl_->size(height, width);
}

void image_writer::set_pixel(COLOR color, int x_position, int y_position) {
  pimpl_->set_pixel(color, x_position, y_position);
}
