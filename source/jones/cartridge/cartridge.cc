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
#include "configuration.hh"
#include "mapper/mapper.hh"
#include "mapper/mapper_view.hh"
#include "memory.hh"

#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

using namespace jones;

namespace ip = boost::interprocess;
namespace fs = boost::filesystem;
namespace jc = jones::configuration;

class file_mapped_cartridge final : public mapped_cartridge {
public:
  explicit file_mapped_cartridge(const char *file_path)
      : header_(create_cartridge_header(file_path)),
        mapped_region_(map_cartridge_file(file_path)) {
    if (valid()) {
      const auto mirror = header_->mirror_type();
      jc::configuration::instance().set(jc::property::PROPERTY_MIRROR_MODE, mirror);
    }
  }

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

class cartridge::impl {
public:
  impl(memory &cpu_memory, memory &ppu_memory) : cpu_memory_(cpu_memory), ppu_memory_(ppu_memory) {}

  ~impl() = default;

  bool attach(char const *const file_path) {
    if (file_path == nullptr) {
      cartridge_.reset();
      mapper_view_.reset();
      cartridge_mapper_.reset();
      cartridge_.reset();
      return true;
    }
    if (fs::exists(file_path)) {
      cartridge_ = std::make_unique<file_mapped_cartridge>(file_path);
      if (cartridge_->valid()) {
        mapper_view_ = std::make_unique<mapper_view>(*cartridge_, cpu_memory_, ppu_memory_);
        cartridge_mapper_ = mapper_factory::get(*mapper_view_);
        return true;
      }
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

  uint8_t peek(uint16_t const address) const {
    if (cartridge_mapper_ != nullptr) {
      return cartridge_mapper_->peek(address);
    }
    return 0;
  }

  uint8_t read(uint16_t const address) const {
    if (cartridge_mapper_ != nullptr) {
      return cartridge_mapper_->read(address);
    }
    return 0;
  }

  void write(uint16_t const address, const uint8_t data) const {
    if (cartridge_mapper_ != nullptr) {
      return cartridge_mapper_->write(address, data);
    }
  }

private:
  memory &cpu_memory_;

  memory &ppu_memory_;

  std::unique_ptr<mapper_view> mapper_view_{};

  std::unique_ptr<file_mapped_cartridge> cartridge_{};

  std::unique_ptr<mapper> cartridge_mapper_{};
};

cartridge::cartridge(memory &cpu_memory, memory &ppu_memory) : impl_(new impl(cpu_memory, ppu_memory)) {}

cartridge::~cartridge() = default;

auto cartridge::attach(char const *const path) -> bool {
  return impl_->attach(path);
}

auto cartridge::print(std::ostream &out) const -> void {
  return impl_->print(out);
}

auto cartridge::dump_prg(std::ostream &out) const -> void {
  return impl_->dump_prg(out);
}

auto cartridge::dump_chr(std::ostream &out) const -> void {
  return impl_->dump_chr(out);
}

auto cartridge::peek(uint16_t const address) const -> uint8_t {
  return impl_->peek(address);
}

auto cartridge::read(uint16_t const address) const -> uint8_t {
  return impl_->read(address);
}

auto cartridge::write(uint16_t const address, uint8_t const data) -> void {
  return impl_->write(address, data);
}
