//
// MIT License
//
// Copyright 2019-2020
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
#include <sstream>
#include <string>

#include "cartridge.hh"
#include "disassemble.hh"
#include "tool.hh"
#include "tool_decoder.hh"

namespace fs = boost::filesystem;
namespace ip = boost::interprocess;

namespace jd = jones::disassemble;
namespace jo = jones;

namespace {

std::string to_hex_string(uint8_t *data, size_t size, size_t width, bool left_to_right = true) {
  std::ostringstream oss;
  if (left_to_right) {
    for (int i = 0; i < static_cast<int>(size); i++) {
      oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<uint16_t>(data[i]);
    }
  } else {
    for (int i = (size - 1); i >= 0; i--) {
      oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<uint16_t>(data[i]);
    }
  }
  for (int i = 0; i < static_cast<int>(width - (size * 2)); i++) {
    oss << " ";
  }
  return oss.str();
}

void dump_code(const fs::path &root_path, const jo::cartridge &rom) {
  boost::ignore_unused(rom);

  const fs::path code_path = root_path / "code";
  fs::create_directories(code_path);
  fs::ofstream code_file{code_path / "prgrom.a65"};

  const fs::path prgrom_path = root_path / "raw" / "prgrom.bin";
  const ip::file_mapping mapped_file(prgrom_path.string().c_str(), ip::mode_t::read_only);
  const ip::mapped_region mapped_region(mapped_file, ip::mode_t::read_only);

  auto const &disassembled_code = jd::disassemble(static_cast<uint8_t *>(mapped_region.get_address()), mapped_region.get_size());
  auto const &disassembled_instructions = disassembled_code.instructions;
  for (auto instruction : disassembled_instructions) {
    auto instruction_address = instruction.address;
    code_file << "   " << to_hex_string(reinterpret_cast<uint8_t *>(&instruction_address), sizeof(instruction.address), 5, false);
    code_file << "   " << to_hex_string(&instruction.length_in_bytes, sizeof(instruction.length_in_bytes), 5);
    code_file << "   " << to_hex_string(instruction.binary.data(), instruction.length_in_bytes, 10);
    code_file << "   " << instruction.opcode << instruction.operand << std::endl;
  }
}

void dump_audio(const fs::path &root_path, const jo::cartridge &rom) {
  boost::ignore_unused(rom);
  const fs::path audio_path = root_path / "audio";
  fs::create_directories(audio_path);
}

void dump_image(const fs::path &root_path, const jo::cartridge &rom) {
  boost::ignore_unused(rom);

  const fs::path image_path = root_path / "image";
  fs::create_directories(image_path);

  const fs::path chrrom_path = root_path / "raw" / "chrrom.bin";
  const ip::file_mapping mapped_file(chrrom_path.string().c_str(), ip::mode_t::read_only);
  const ip::mapped_region mapped_region(mapped_file, ip::mode_t::read_only);

  jones::tool::decode_chrrom(static_cast<uint8_t *>(mapped_region.get_address()), mapped_region.get_size(), image_path.string().c_str());
}

void dump_raw(const fs::path &root_path, const jo::cartridge &rom) {
  const fs::path raw_path = root_path / "raw";
  fs::create_directories(raw_path);
  {
    const fs::path chrom_path = raw_path / "chrrom.bin";
    std::ofstream chrom_file{chrom_path.string()};
    rom.dump_chr(chrom_file);
  }
  {
    const fs::path prgrom_path = raw_path / "prgrom.bin";
    std::ofstream prgrom_file{prgrom_path.string()};
    rom.dump_prg(prgrom_file);
  }
}

void dump_header(const fs::path &root_path, const jo::cartridge &rom) {
  const fs::path header_path = root_path / "rom_header.txt";
  std::ofstream header_file{header_path.string()};
  rom.print(header_file);
}

} // namespace

namespace jones::tool {

int dump(const char *file_path, const char *output_path) {
  memory cpu_memory{"cpu"};
  memory ppu_memory{"ppu"};
  cartridge rom = cartridge(cpu_memory, ppu_memory);
  if (!rom.attach(file_path)) {
    std::cerr << "error: must specify valid rom file" << std::endl;
    return -1;
  }

  dump_header(output_path, rom);
  dump_raw(output_path, rom);
  dump_code(output_path, rom);
  dump_audio(output_path, rom);
  dump_image(output_path, rom);

  return 0;
}

} // namespace jones::tool
