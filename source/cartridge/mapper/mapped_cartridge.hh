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

  virtual void print(std::ostream &ostream) const = 0;

  virtual bool valid() const = 0;

  virtual size_t version() const = 0;

  virtual uint16_t prgrom_offset() const = 0;

  virtual uint16_t prgrom_size() const = 0;

  virtual uint8_t prgrom_count() const = 0;

  virtual uint16_t chrrom_offset() const = 0;

  virtual uint16_t chrrom_size() const = 0;

  virtual uint8_t chrrom_count() const = 0;

  virtual uint8_t mapper_number() const = 0;

  virtual bool mirroring() const = 0;
};

class mapped_cartridge {
public:
  virtual ~mapped_cartridge() = default;

  virtual bool valid() const = 0;

  virtual size_t size() const = 0;

  virtual uint8_t *address() const = 0;

  virtual mapped_cartridge_header *header() const = 0;
};

} // namespace jones

#endif // JONES_CARTRIDGE_MAPPER_MAPPED_CARTRIDGE_HH
