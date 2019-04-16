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
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iomanip>
#include <iostream>

#include "cartridge.hh"
#include "mapper/mapper.hh"
#include "memory.hh"

#define ROM_HEADER_CONSTANT 0x1A53454E

using namespace jones;

namespace ip = boost::interprocess;

namespace {

void print_hex_integer(std::ostream &out, const char *label, const uint32_t value) {
  out << "  " << label << std::hex << static_cast<uint32_t>(value) << std::endl;
}

void print_hex_integer(std::ostream &out, const char *label, const uint8_t value) {
  out << "  " << label << std::hex << static_cast<uint16_t>(value) << std::endl;
}

} // namespace

class cartridge::impl {
public:
  impl() : rom_file_header_(), cartridge_file_(), cartridge_mapper_() {}

  ~impl() = default;

  bool attach(const char *file_path) {
    std::ifstream rom_file_ = std::ifstream(file_path, std::ifstream::binary);
    if (rom_file_.good()) {
      rom_file_.read((char *)&rom_file_header_, sizeof(rom_file_header_));
      if (rom_file_header_.constants == ROM_HEADER_CONSTANT) {
        cartridge_file_ = std::make_unique<mapped_cartridge_file>(file_path);
        cartridge_mapper_ = mappers::get(get_mapper_number());
        return true;
      }
    }
    return false;
  }

  int get_header_version() const {
    return (rom_file_header_.version == 2) ? 2 : 1;
  }

  uint16_t get_prgrom_offset() const {
    return sizeof(rom_file_header_) + (rom_file_header_.has_trainer == 0 ? 0 : (sizeof(uint8_t) * 512));
  }

  uint16_t get_prgrom_size() const {
    return static_cast<uint16_t>(rom_file_header_.prgrom_size * 16384);
  }

  uint16_t get_chrrom_offset() const {
    return get_prgrom_offset() + get_prgrom_size();
  }

  uint16_t get_chrrom_size() const {
    return static_cast<uint16_t>(rom_file_header_.chrrom_size * 8192);
  }

  uint8_t get_mapper_number() const {
    return (rom_file_header_.upper_mapper_nibble << 4U) | (rom_file_header_.lower_mapper_nibble);
  }

  void print_header(std::ostream &out) const {
    out << "***********************************************" << std::endl;
    out << "  Nes Rom Header" << std::endl;
    out << "***********************************************" << std::endl;
    print_hex_integer(out, "constants: ", rom_file_header_.constants);
    print_hex_integer(out, "prgrom_size: ", rom_file_header_.prgrom_size);
    print_hex_integer(out, "chrrom_size: ", rom_file_header_.chrrom_size);
    print_hex_integer(out, "has_mirroring: ", rom_file_header_.has_mirroring);
    print_hex_integer(out, "contains_battery_backed_prgram: ", rom_file_header_.contains_battery_backed_prgram);
    print_hex_integer(out, "has_trainer: ", rom_file_header_.has_trainer);
    print_hex_integer(out, "ignore_mirroring_control: ", rom_file_header_.ignore_mirroring_control);
    print_hex_integer(out, "lower_mapper_nibble: ", rom_file_header_.lower_mapper_nibble);
    print_hex_integer(out, "is_vs_unisystem: ", rom_file_header_.is_vs_unisystem);
    print_hex_integer(out, "is_play_choice: ", rom_file_header_.is_play_choice);
    print_hex_integer(out, "version: ", rom_file_header_.version);
    print_hex_integer(out, "upper_mapper_nibble: ", rom_file_header_.upper_mapper_nibble);
    print_hex_integer(out, "prgram_size: ", rom_file_header_.prgram_size);
    print_hex_integer(out, "which_tv_system_one: ", rom_file_header_.which_tv_system_one);
    print_hex_integer(out, "which_tv_system_two: ", rom_file_header_.which_tv_system_two);
    print_hex_integer(out, "is_prgram_present: ", rom_file_header_.is_prgram_present);
    print_hex_integer(out, "has_bus_conflicts: ", rom_file_header_.has_bus_conflicts);
    out << "***********************************************" << std::endl;
  }

  uint8_t read(uint16_t address) const {
    if (cartridge_file_) {
      return *(cartridge_file_->address() + address);
    }
    return 0;
  }

  void write(uint16_t address, uint8_t data) {
    if (cartridge_file_) {
      *(cartridge_file_->address() + address) = data;
    }
  }

private:
  class mapped_cartridge_file {
  public:
    mapped_cartridge_file(const char *file_path)
        : mapped_region_(map_cartridge_file(file_path)) {}

    ~mapped_cartridge_file() = default;

    size_t size() {
      return mapped_region_.get_size();
    }

    uint8_t *address() {
      return static_cast<uint8_t *>(mapped_region_.get_address());
    }

  private:
    static ip::mapped_region map_cartridge_file(const char *file_path) {
      const ip::file_mapping mapped_file(file_path, ip::mode_t::read_only);
      return ip::mapped_region(mapped_file, ip::mode_t::read_only);
    }

    const ip::mapped_region mapped_region_;
  };

  struct rom_header {
    uint32_t constants;
    uint8_t prgrom_size;
    uint8_t chrrom_size;
    uint8_t has_mirroring : 1;
    uint8_t contains_battery_backed_prgram : 1;
    uint8_t has_trainer : 1;
    uint8_t ignore_mirroring_control : 1;
    uint8_t lower_mapper_nibble : 4;
    uint8_t is_vs_unisystem : 1;
    uint8_t is_play_choice : 1;
    uint8_t version : 2;
    uint8_t upper_mapper_nibble : 4;
    uint8_t prgram_size;
    uint8_t which_tv_system_one : 1;
    unsigned int : 7;
    uint8_t which_tv_system_two : 2;
    unsigned int : 2;
    uint8_t is_prgram_present : 1;
    uint8_t has_bus_conflicts : 1;
    unsigned int : 2;
    uint8_t ignored[5];
  };

  rom_header rom_file_header_;

  std::unique_ptr<mapped_cartridge_file> cartridge_file_;

  std::unique_ptr<mapper> cartridge_mapper_;
};

cartridge::cartridge() : impl_(new impl()) {}

cartridge::~cartridge() = default;

bool cartridge::attach(const char *file_path) {
  return impl_->attach(file_path);
}

int cartridge::get_header_version() const {
  return impl_->get_header_version();
}

void cartridge::print_header(std::ostream &out) const {
  impl_->print_header(out);
}

uint16_t cartridge::get_prgrom_offset() const {
  return impl_->get_prgrom_offset();
}

uint16_t cartridge::get_prgrom_size() const {
  return impl_->get_prgrom_size();
}

uint16_t cartridge::get_chrrom_offset() const {
  return impl_->get_chrrom_offset();
}

uint16_t cartridge::get_chrrom_size() const {
  return impl_->get_chrrom_size();
}

uint8_t cartridge::get_mapper_number() const {
  return impl_->get_mapper_number();
}

uint8_t cartridge::read(uint16_t address) const {
  return impl_->read(address);
}

void cartridge::write(uint16_t address, uint8_t data) {
  impl_->write(address, data);
}