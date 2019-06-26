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
#ifndef JONES_CARTRIDGE_MAPPER_MAPPED_CARTRIDGE_HH
#define JONES_CARTRIDGE_MAPPER_MAPPED_CARTRIDGE_HH

namespace jones {

class mapped_cartridge_header {
public:
  mapped_cartridge_header() = default;

  virtual ~mapped_cartridge_header() = default;

  virtual auto print(std::ostream &out) const -> void = 0;

  virtual auto valid() const -> bool = 0;

  virtual auto version() const -> size_t = 0;

  virtual auto prgrom_offset() const -> uint16_t = 0;

  virtual auto prgrom_size() const -> uint16_t = 0;

  virtual auto prgrom_count() const -> uint8_t = 0;

  virtual auto chrrom_offset() const -> uint16_t = 0;

  virtual auto chrrom_size() const -> uint16_t = 0;

  virtual auto chrrom_count() const -> uint8_t = 0;

  virtual auto mapper_number() const -> uint8_t = 0;

  virtual auto has_mirroring() const -> bool = 0;

  virtual auto mirror_type() const -> uint8_t = 0;
};

class mapped_cartridge {
public:
  virtual ~mapped_cartridge() = default;

  virtual auto valid() const -> bool = 0;

  virtual auto size() const -> size_t = 0;

  virtual auto address() const -> uint8_t * = 0;

  virtual auto header() const -> mapped_cartridge_header * = 0;
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPER_MAPPED_CARTRIDGE_HH
