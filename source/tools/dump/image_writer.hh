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
#ifndef JONES_TOOL_IMAGE_WRITER_HH
#define JONES_TOOL_IMAGE_WRITER_HH

#include <cstdint>

namespace jones::tool {

enum class COLOR;

class image_writer {
public:
  image_writer();

  ~image_writer();

  image_writer(image_writer &&) noexcept;

  image_writer &operator=(image_writer &&) noexcept;

  void write(const char *file_path);

  void size(int height, int width);

  void set_pixel(COLOR color, int x_postion, int y_position);

private:
  class image_writer_impl;

  std::unique_ptr<image_writer_impl> pimpl_;
};

} // namespace jones::tool

#endif // JONES_TOOL_IMAGE_WRITER_HH
