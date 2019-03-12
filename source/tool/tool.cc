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
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "cartridge.hh"
#include "disassemble.hh"

namespace fs = boost::filesystem;
namespace ip = boost::interprocess;
namespace po = boost::program_options;

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

void dump_code(const fs::path &root_path, const jo::cartridge &rom, const uint8_t *const base_address, const size_t length_in_bytes) {
  boost::ignore_unused(rom);
  boost::ignore_unused(length_in_bytes);

  const fs::path code_path = root_path / "code";
  fs::create_directories(code_path);
  fs::ofstream code_file{code_path / "prgrom.a65"};

  const uint8_t *const prgrom_base_address = base_address + rom.get_prgrom_offset();
  const auto &disassembled_code = jd::disassemble(const_cast<uint8_t *>(prgrom_base_address), rom.get_prgrom_size());
  const auto &disassembled_instructions = disassembled_code.instructions;
  for (auto instruction : disassembled_instructions) {
    auto instruction_address = instruction.address + rom.get_prgrom_offset();
    code_file << "   " << to_hex_string(reinterpret_cast<uint8_t *>(&instruction_address), sizeof(instruction.address), 5, false);
    code_file << "   " << to_hex_string(&instruction.length_in_bytes, sizeof(instruction.length_in_bytes), 5);
    code_file << "   " << to_hex_string(instruction.binary.data(), instruction.length_in_bytes, 10);
    code_file << "   " << instruction.opcode << instruction.operand << std::endl;
  }
}

void dump_audio(const fs::path &root_path, const jo::cartridge &rom, const uint8_t *const base_address, const size_t length_in_bytes) {
  boost::ignore_unused(rom);
  boost::ignore_unused(base_address);
  boost::ignore_unused(length_in_bytes);

  const fs::path audio_path = root_path / "audio";
  fs::create_directories(audio_path);
}

void dump_image(const fs::path &root_path, const jo::cartridge &rom, const uint8_t *const base_address, const size_t length_in_bytes) {
  boost::ignore_unused(rom);
  boost::ignore_unused(base_address);
  boost::ignore_unused(length_in_bytes);

  const fs::path image_path = root_path / "image";
  fs::create_directories(image_path);
}
} // namespace

int main(int argc, char *argv[]) {

  std::string file_argument;
  std::string output_argument;

  try {
    po::options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")("file,f", po::value<std::string>(&file_argument)->required(), "file path to rom")("output,o", po::value<std::string>(&output_argument)->default_value("out"), "directory path to write output");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const po::error &ex) {
    std::cerr << ex.what() << '\n';
    return -1;
  }

  const fs::path file_path(file_argument);
  if (!fs::exists(file_path)) {
    std::cerr << "file path does not exist; please check path" << std::endl;
    return -2;
  }

  if (fs::is_directory(file_path)) {
    std::cerr << "file path is a directory; please check path" << std::endl;
    return -3;
  }

  const fs::path output_path(output_argument);
  if (fs::exists(output_path) && !fs::is_directory(output_path)) {
    std::cerr << "output directory path is a file; please check path : " << output_path << std::endl;
    return -4;
  }

  if (!fs::exists(output_path) && !fs::create_directories(output_path)) {
    std::cerr << "unable to create output directory path; please check path : " << output_path << std::endl;
    return -5;
  }

  jones::cartridge rom(file_path.string());
  if (!rom.is_valid()) {
    std::cerr << "error: must specify valid rom file" << std::endl;
    return -6;
  }

  rom.print_header(std::cout);

  const ip::file_mapping mapped_file(file_path.c_str(), ip::mode_t::read_only);
  const ip::mapped_region mapped_region(mapped_file, ip::mode_t::read_only);
  const auto &mapped_base_address = static_cast<uint8_t const *>(mapped_region.get_address());
  const auto &mapped_size = mapped_region.get_size();

  dump_code(output_path, rom, mapped_base_address, mapped_size);
  dump_audio(output_path, rom, mapped_base_address, mapped_size);
  dump_image(output_path, rom, mapped_base_address, mapped_size);

  return 0;
}
