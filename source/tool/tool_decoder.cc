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
#include <iomanip>
#include <sstream>

#include "image_color.hh"
#include "image_writer.hh"
#include "tool_decoder.hh"

namespace fs = boost::filesystem;

namespace jones::tool {

void decode_chrrom(uint8_t *base_address, size_t length_in_bytes, const char *out_path) {
  boost::ignore_unused(base_address);
  boost::ignore_unused(length_in_bytes);
  boost::ignore_unused(out_path);

  const fs::path image_path{out_path};

  //
  // we expect that chrrom is byte aligned since this should always
  // be the case.
  //
  BOOST_ASSERT(length_in_bytes % 8 == 0);

  const auto tile_count = length_in_bytes / 16;
  const auto tile_row_count = tile_count * 2;
  auto file_count = 0;
  for (size_t i = 0; i < tile_row_count;) {
    std::stringstream file_name;
    file_name << "tile_" << std::setfill('0') << std::setw(3) << file_count++ << ".png";
    const fs::path file_path = image_path / file_name.str();

    jones::tool::image_writer writer;
    writer.size(8, 8);
    for (size_t r = 0; r < std::numeric_limits<uint8_t>::digits; r++) {

      const uint8_t tile_row_first_layer = base_address[i++];
      const uint8_t tile_row_second_layer = base_address[i++];

      for (uint8_t b = 0; b < std::numeric_limits<uint8_t>::digits; b++) {
        uint8_t bit_first_layer = (tile_row_first_layer & (1 << b)) != 0 ? 1 : 0;
        uint8_t bit_second_layer = (tile_row_second_layer & (1 << b)) != 0 ? 2 : 0;
        const uint8_t value = bit_first_layer + bit_second_layer;
        switch (value) {
        case 0:
          writer.set_pixel(COLOR::BLACK, b, r);
          break;
        case 1:
          writer.set_pixel(COLOR::WHITE, b, r);
          break;
        case 2:
          writer.set_pixel(COLOR::LIGHT_GRAY, b, r);
          break;
        case 3:
          writer.set_pixel(COLOR::DARK_GRAY, b, r);
          break;
        default:
          break;
        }
      }
    }
    writer.write(file_path.string().c_str());
  }
}

} // namespace jones::tool
