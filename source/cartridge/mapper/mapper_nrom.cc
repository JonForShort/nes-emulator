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
#include "mapper_nrom.hh"

using namespace jones;

mapper_nrom::mapper_nrom(const jones::cartridge &cartridge)
    : mapper(cartridge),
      type_(cartridge.get_prgrom_size() == 1 ? nrom_type::NROM_128 : nrom_type::NROM_256),
      mirroring_type_(cartridge.has_mirroring() ? nrom_mirroring_type::NROM_VERTICAL : nrom_mirroring_type::NROM_HORIZONAL) {}

uint8_t mapper_nrom::read(const uint16_t address) {
  return get_cartridge().read(address);
}

void mapper_nrom::write(const uint16_t address, const uint8_t data) {
  get_cartridge().write(address, data);
}