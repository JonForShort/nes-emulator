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
#include <boost/filesystem.hpp>

#include "png_writer.h"
#include "tool_decoder.hh"

namespace fs = boost::filesystem;

namespace jones::tool {

void decode_chrrom(uint8_t *base_address, size_t length_in_bytes, const char *out_path) {
  boost::ignore_unused(base_address);
  boost::ignore_unused(length_in_bytes);
  boost::ignore_unused(out_path);

  const fs::path image_path{out_path};
  const fs::path file_path = image_path / "redsquare.png";

  jones::tool::png_writer writer;
  writer.size(512, 512);
  writer.set_pixel(COLOR::WHITE, 50, 500);
  writer.write(file_path.string().c_str());

  //  for (size_t i = 0; i < length_in_bytes; i++) {
  //    uint8_t tile_row_first_layer = base_address[i++];
  //    uint8_t tile_row_second_layer = base_address[i++];
  //  }
}

} // namespace jones::tool