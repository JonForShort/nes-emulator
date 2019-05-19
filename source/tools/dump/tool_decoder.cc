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

using namespace jones;

namespace fs = boost::filesystem;

namespace {

std::string build_file_path(const char *out_path, const int id) {
  std::stringstream file_name;
  file_name << "tile_" << std::setfill('0') << std::setw(3) << id << ".png";
  return (fs::path{out_path} / file_name.str()).string();
}

constexpr uint8_t pixels_per_tile = 8;

constexpr tool::COLOR tile_palette[] = {tool::COLOR::BLACK, tool::COLOR::WHITE, tool::COLOR::LIGHT_GRAY, tool::COLOR::DARK_GRAY};

} // namespace

void tool::decode_chrrom(const uint8_t *base_address, const size_t length_in_bytes, const char *out_path) {
  BOOST_ASSERT(length_in_bytes % 8 == 0);
  auto file_count = 0;
  for (size_t i = 0; i < length_in_bytes;) {
    const auto file_path = build_file_path(out_path, file_count++);
    jones::tool::image_writer writer;
    writer.size(pixels_per_tile, pixels_per_tile);
    for (auto r = 0; r < pixels_per_tile; r++) {
      const uint8_t first_layer = base_address[i++];
      const uint8_t second_layer = base_address[i++];
      for (auto b = 0; b < pixels_per_tile; b++) {
        const uint8_t first_layer_value = (first_layer & (1 << b)) != 0 ? 1 : 0;
        const uint8_t second_layer_value = (second_layer & (1 << b)) != 0 ? 2 : 0;
        const uint8_t value = first_layer_value + second_layer_value;
        writer.set_pixel(tile_palette[value], b, r);
      }
    }
    writer.write(file_path.c_str());
  }
}
