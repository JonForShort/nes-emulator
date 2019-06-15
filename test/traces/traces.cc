#include <utility>

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
#define BOOST_TEST_MODULE test_suite_traces

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/test/unit_test.hpp>
#include <iomanip>
#include <iostream>
#include <vector>

#include "cartridge.hh"
#include "debugger.hh"
#include "log.hh"
#include "nes.hh"

namespace bt = boost::unit_test;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace bs = boost::iostreams;

using format = boost::format;

namespace {

void gunzip(const std::string &input_file, const std::string &output_file) {
  std::ifstream compressed(input_file, std::ios::in | std::ios::binary);
  std::ofstream decompressed(output_file, std::ios::out | std::ios::binary | std::ios::app);

  bs::filtering_streambuf<bs::input> compressed_stream;
  compressed_stream.push(bs::gzip_decompressor());
  compressed_stream.push(compressed);
  bs::copy(compressed_stream, decompressed);
}

std::string fix_instruction(std::string instruction) {
  std::vector<std::string> instruction_pieces;
  boost::split(instruction_pieces, instruction, boost::is_any_of("=@"));
  const auto fixed_unofficial = boost::replace_all_copy(instruction_pieces[0], "*", "");
  const auto fixed_opcodes = boost::replace_all_copy(fixed_unofficial, "ISB", "ISC");
  return fixed_opcodes;
}

void check_trace_files(const std::string &trace_path, const std::string &result_path) {
  std::ifstream trace_file{trace_path};
  std::ifstream result_file{result_path};
  int line_count = 0;
  for (std::string trace_line, result_line;
       std::getline(trace_file, trace_line) && std::getline(result_file, result_line);
       line_count++) {

    const auto trace_address = boost::trim_copy(trace_line.substr(0, 4));
    const auto result_address = boost::trim_copy(result_line.substr(0, 4));
    BOOST_CHECK_MESSAGE(trace_address == result_address,
                        format("line [%d] : check [address] : expected [%s] found [%s]") % line_count % result_address % trace_address);

    const auto trace_binary = boost::trim_copy(trace_line.substr(6, 10));
    const auto result_binary = boost::trim_copy(boost::replace_all_copy(result_line.substr(6, 10), "*", ""));
    BOOST_CHECK_MESSAGE(trace_binary == result_binary,
                        format("line [%d] : check [binary] : expected [%s] found [%s]") % line_count % result_binary % trace_binary);

    const auto trace_instruction = boost::trim_copy(trace_line.substr(15, 16));
    const auto result_instruction = boost::trim_copy(fix_instruction(result_line.substr(15, 16)));
    BOOST_CHECK_MESSAGE(trace_instruction == result_instruction,
                        format("line [%d] : check [instruction] : expected [%s] found [%s]") % line_count % result_instruction % trace_instruction);

    const auto trace_register_a = boost::trim_copy(trace_line.substr(50, 2));
    const auto result_register_a = boost::trim_copy(result_line.substr(50, 2));
    BOOST_CHECK_MESSAGE(trace_register_a == result_register_a,
                        format("line [%d] : check [a] : expected [%s] found [%s]") % line_count % result_register_a % trace_register_a);

    const auto trace_register_x = boost::trim_copy(trace_line.substr(55, 2));
    const auto result_register_x = boost::trim_copy(result_line.substr(55, 2));
    BOOST_CHECK_MESSAGE(trace_register_x == result_register_x,
                        format("line [%d] : check [x] : expected [%s] found [%s]") % line_count % result_register_x % trace_register_x);

    const auto trace_register_y = boost::trim_copy(trace_line.substr(60, 2));
    const auto result_register_y = boost::trim_copy(result_line.substr(60, 2));
    BOOST_CHECK_MESSAGE(trace_register_y == result_register_y,
                        format("line [%d] : check [y] : expected [%s] found [%s]") % line_count % result_register_y % trace_register_y);

    const auto trace_register_p = boost::trim_copy(trace_line.substr(65, 2));
    const auto result_register_p = boost::trim_copy(result_line.substr(65, 2));
    BOOST_CHECK_MESSAGE(trace_register_p == result_register_p,
                        format("line [%d] : check [p] : expected [%s] found [%s]") % line_count % result_register_p % trace_register_p);

    const auto trace_register_sp = boost::trim_copy(trace_line.substr(71, 2));
    const auto result_register_sp = boost::trim_copy(result_line.substr(71, 2));
    BOOST_CHECK_MESSAGE(trace_register_sp == result_register_sp,
                        format("line [%d] : check [sp] : expected [%s] found [%s]") % line_count % result_register_sp % trace_register_sp);

    const auto trace_ppu = boost::trim_copy(trace_line.substr(74, 12));
    const auto result_ppu = boost::trim_copy(result_line.substr(74, 12));
    BOOST_CHECK_MESSAGE(trace_ppu == result_ppu,
                        format("line [%d] : check [ppu] : expected [%s] found [%s]") % line_count % result_ppu % trace_ppu);

    const auto trace_cpu = boost::trim_copy(trace_line.substr(86));
    const auto result_cpu = boost::trim_copy(result_line.substr(86));
    BOOST_CHECK_MESSAGE(trace_cpu == result_cpu,
                        format("line [%d] : check [cpu] : expected [%s] found [%s]") % line_count % result_cpu % trace_cpu);
  }
}

bool build_trace_file(const fs::path &trace_path, const fs::path trace_root_path, const std::string &trace_name) {
  std::vector<std::string> trace_files;
  for (auto &entry : boost::make_iterator_range(fs::directory_iterator(trace_root_path), {})) {
    const auto &trace_filename = fs::path(entry).filename().stem();
    if (ba::starts_with(trace_filename.c_str(), trace_name)) {
      trace_files.emplace_back(trace_filename.c_str());
    }
  }
  std::sort(trace_files.begin(), trace_files.end());
  for (const auto &trace_file : trace_files) {
    const auto compressed_trace_file = trace_root_path / (trace_file + ".gz");
    if (!fs::exists(compressed_trace_file)) {
      LOG_ERROR << "unable to find expected compressed trace file";
      return false;
    }
    gunzip(compressed_trace_file.string(), trace_path.string());
  }
  return true;
}

size_t get_line_count(const std::string &file_path) {
  std::ifstream in_file(file_path);
  std::string line;
  size_t count = 0;
  while (std::getline(in_file, line)) {
    count++;
  }
  return count;
}

void add_trace_entry(const fs::path &trace_path, const jones::nes_state &nes_state) {
  std::ofstream trace_file(trace_path, std::ios::app);

  trace_file << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << nes_state.registers.PC << "  ";

  for (const auto &i : nes_state.instruction_bytes) {
    trace_file << std::right << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<uint16_t>(i) << " ";
  }

  const size_t instruction_bytes_padding = 10 - (nes_state.instruction_bytes.size() * 3);
  for (size_t i = 0; i < instruction_bytes_padding; i++) {
    trace_file << " ";
  }

  trace_file << nes_state.instruction;

  const size_t instruction_padding = 32 - nes_state.instruction.length();
  for (size_t i = 0; i < instruction_padding; i++) {
    trace_file << " ";
  }

  trace_file << "A:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.registers.A) << " ";
  trace_file << "X:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.registers.X) << " ";
  trace_file << "Y:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.registers.Y) << " ";
  trace_file << "P:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.registers.SR) << " ";
  trace_file << "SP:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.registers.SP) << " ";

  trace_file << "PPU:";

  trace_file << std::dec << std::right << std::setw(3) << std::setfill(' ') << nes_state.ppu_cycle << ",";

  trace_file << std::dec << std::right << std::setw(3) << std::setfill(' ') << nes_state.ppu_scanline << " ";

  trace_file << "CYC:" << std::dec << nes_state.cpu_cycle;

  trace_file << std::endl;
}

class trace_listener : public jones::nes_listener {
public:
  explicit trace_listener(std::string trace_path) : trace_path_(std::move(trace_path)) {}

  ~trace_listener() override = default;

  auto on_event(jones::nes_listener::event event, jones::nes_state state) -> void override {
    switch (event) {
    case event::ON_RESET:
    case event::ON_RUN_FINISHED:
    case event::ON_RUN_PAUSED:
      break;
    default:
      add_trace_entry(trace_path_, state);
    }
  }

private:
  std::string trace_path_;
};

} // namespace

BOOST_AUTO_TEST_CASE(test_suite_traces) {
  const auto argc = bt::framework::master_test_suite().argc;
  if (argc <= 1) {
    BOOST_FAIL("missing required test arguments; specify root trace path");
    return;
  }
  const auto argv = bt::framework::master_test_suite().argv;
  const auto root_trace_path = argv[1];

  const auto &root_trace_iterator = fs::directory_iterator(root_trace_path);
  for (auto &trace_directory_entry : boost::make_iterator_range(root_trace_iterator, {})) {
    const auto trace_directory_path = fs::path(trace_directory_entry);
    if (is_directory(trace_directory_path)) {
      jones::nes nes;
      jones::debugger debugger(nes);

      const auto trace_test_path = fs::temp_directory_path() / fs::unique_path();
      const auto trace_result_path = fs::temp_directory_path() / fs::unique_path();
      const auto trace_name = trace_directory_path.filename().string() + ".trace";

      const auto nes_listener = std::make_unique<trace_listener>(trace_test_path.string());
      debugger.set_listener(nes_listener.get());

      const auto is_trace_file_built = build_trace_file(trace_result_path, trace_directory_path, trace_name);
      if (is_trace_file_built) {
        const auto rom_path = trace_directory_path / (trace_directory_path.filename().string() + ".nes");
        const auto rom_loaded = nes.load(rom_path.c_str());
        if (rom_loaded) {
          const auto step_count = get_line_count(trace_result_path.string());
          nes.run(step_count);
          check_trace_files(trace_test_path.string(), trace_result_path.string());
          continue;
        }
      }
      BOOST_FAIL("unable to complete test");
    }
  }
}
