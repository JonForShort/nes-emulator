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

#include "boost_gil.h"
#include "png_writer.h"

namespace bg = boost::gil;

class png_writer::png_writer_impl {
public:
  void write(const char *file_path) {
    boost::ignore_unused(file_path);
  }

  void size(int height, int width) {
    boost::ignore_unused(height);
    boost::ignore_unused(width);
  }

  void set_pixel(COLOR pixel_color) {
    boost::ignore_unused(pixel_color);
  }
};

png_writer::png_writer() : pimpl_{std::make_unique<png_writer_impl>()} {
}

png_writer::~png_writer() = default;

png_writer::png_writer(png_writer &&) noexcept = default;

png_writer &png_writer::operator=(png_writer &&) noexcept = default;

void png_writer::write(const char *file_path) {
  pimpl_->write(file_path);
}

void png_writer::size(int height, int width) {
  pimpl_->size(height, width);
}

void png_writer::set_pixel(COLOR pixel_color) {
  pimpl_->set_pixel(pixel_color);
}