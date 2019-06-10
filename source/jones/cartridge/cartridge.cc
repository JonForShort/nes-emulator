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
#include "cartridge.hh"
#include "cartridge_header.hh"
#include "mapper/mapper.hh"
#include "memory.hh"

#include <boost/core/ignore_unused.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iomanip>
#include <iostream>

using namespace jones;

namespace ip = boost::interprocess;

class file_mapped_cartridge final : public mapped_cartridge {
public:
  explicit file_mapped_cartridge(const char *file_path)
      : header_(create_cartridge_header(file_path)),
        mapped_region_(map_cartridge_file(file_path)) {}

  ~file_mapped_cartridge() override = default;

  size_t size() const override {
    return mapped_region_.get_size();
  }

  uint8_t *address() const override {
    return static_cast<uint8_t *>(mapped_region_.get_address());
  }

  bool valid() const override {
    return header() != nullptr && header()->valid();
  }

  mapped_cartridge_header *header() const override {
    return header_.get();
  }

private:
  static ip::mapped_region map_cartridge_file(const char *file_path) {
    const ip::file_mapping mapped_file(file_path, ip::mode_t::read_only);
    return ip::mapped_region(mapped_file, ip::mode_t::read_only);
  }

  static std::unique_ptr<cartridge_header> create_cartridge_header(const char *file_path) {
    std::ifstream file = std::ifstream(file_path, std::ifstream::binary);
    return std::make_unique<cartridge_header>(std::move(file));
  }

  const std::unique_ptr<cartridge_header> header_;

  const ip::mapped_region mapped_region_;
};

class memory_mapped_chrom : public memory_mappable {
public:
  explicit memory_mapped_chrom(mapper *mapper) : mapper_(mapper) {}

  ~memory_mapped_chrom() override = default;

  auto start_address() -> uint16_t override {
    return 0x0000;
  }

  auto end_address() -> uint16_t override {
    return 0x1FFF;
  }

  auto read(const uint16_t address) -> uint8_t override {
    return mapper_->read_chr(address);
  }

  auto write(const uint16_t address, const uint8_t data) -> void override {
    mapper_->write_chr(address, data);
  }

private:
  mapper *const mapper_;
};

class cartridge::impl {
public:
  impl(memory &cpu_memory, memory &ppu_memory) : cpu_memory_(cpu_memory), ppu_memory_(ppu_memory) {}

  ~impl() = default;

  bool attach(const char *file_path) {
    cartridge_ = std::make_unique<file_mapped_cartridge>(file_path);
    if (cartridge_->valid()) {
      cartridge_mapper_ = mappers::get(*cartridge_);
      ppu_memory_.map(std::make_unique<memory_mapped_chrom>(cartridge_mapper_.get()));
      return true;
    }
    return false;
  }

  void print(std::ostream &out) const {
    if (cartridge_) {
      cartridge_->header()->print(out);
    }
  }

  void dump_prg(std::ostream &out) {
    if (cartridge_) {
      const auto base_address = reinterpret_cast<char *>(cartridge_->header()->prgrom_offset() + cartridge_->address());
      const auto size = cartridge_->header()->prgrom_size();
      out.write(base_address, size);
    }
  }

  void dump_chr(std::ostream &out) {
    if (cartridge_) {
      const auto base_address = reinterpret_cast<char *>(cartridge_->header()->chrrom_offset() + cartridge_->address());
      const auto size = cartridge_->header()->prgrom_size();
      out.write(base_address, size);
    }
  }

  uint8_t read(const uint16_t address) const {
    if (address >= 0x8000U) {
      return read_prg(address);
    }
    return 0;
  }

  void write(const uint16_t address, const uint8_t data) const {
    if (address >= 0x8000U) {
      return write_prg(address, data);
    }
  }

private:
  uint8_t read_prg(const uint16_t address) const {
    if (cartridge_mapper_) {
      return cartridge_mapper_->read_prg(address);
    }
    return 0;
  }

  void write_prg(const uint16_t address, const uint8_t data) const {
    if (cartridge_mapper_) {
      cartridge_mapper_->write_prg(address, data);
    }
  }

  uint8_t read_chr(const uint16_t address) const {
    if (cartridge_mapper_) {
      return cartridge_mapper_->read_chr(address);
    }
    return 0;
  }

  void write_chr(const uint16_t address, const uint8_t data) const {
    if (cartridge_mapper_) {
      cartridge_mapper_->write_chr(address, data);
    }
  }

private:
  memory &cpu_memory_;

  memory &ppu_memory_;

  std::unique_ptr<file_mapped_cartridge> cartridge_{};

  std::unique_ptr<mapper> cartridge_mapper_{};
};

cartridge::cartridge(memory &cpu_memory, memory &ppu_memory) : impl_(new impl(cpu_memory, ppu_memory)) {}

cartridge::~cartridge() = default;

bool cartridge::attach(const char *path) {
  return impl_->attach(path);
}

void cartridge::print(std::ostream &out) const {
  impl_->print(out);
}

void cartridge::dump_prg(std::ostream &out) const {
  return impl_->dump_prg(out);
}

void cartridge::dump_chr(std::ostream &out) const {
  return impl_->dump_chr(out);
}

uint8_t cartridge::read(const uint16_t address) const {
  return impl_->read(address);
}

void cartridge::write(const uint16_t address, const uint8_t data) const {
  return impl_->write(address, data);
}
