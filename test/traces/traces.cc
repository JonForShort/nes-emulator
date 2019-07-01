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
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/test/results_collector.hpp>
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
namespace pt = boost::property_tree;

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

std::map<std::string, std::string> build_trace_details_map(const std::string &trace) {
  std::map<std::string, std::string> result;
  std::vector<std::string> pieces;
  boost::split(pieces, trace, boost::is_any_of(" "), boost::token_compress_on);
  for (auto &i : pieces) {
    std::vector<std::string> piece_section;
    boost::split(piece_section, i, boost::is_any_of(":"));
    if (piece_section.size() == 2) {
      result[piece_section[0]] = piece_section[1];
    }
  }
  return result;
}

void stop_test_if_failed() {
  const auto test_id = bt::framework::current_test_case().p_id;
  BOOST_REQUIRE(bt::results_collector.results(test_id).passed());
}

void check_trace_files(const std::string &trace_path, const std::string &result_path) {
  BOOST_TEST_MESSAGE(format("check trace file : trace [%s] result [%s]") % trace_path % result_path);

  std::ifstream trace_file{trace_path};
  std::ifstream result_file{result_path};
  int line_count = 0;
  for (std::string trace_line, result_line;
       std::getline(trace_file, trace_line) && std::getline(result_file, result_line);
       line_count++) {

    const auto trace_address = boost::trim_copy(trace_line.substr(0, 5));
    const auto result_address = boost::trim_copy(result_line.substr(0, 5));
    BOOST_CHECK_MESSAGE(trace_address == result_address,
                        format("line [%d] : check [address] : expected [%s] found [%s]") % line_count % result_address % trace_address);

    const auto trace_binary = boost::trim_copy(trace_line.substr(5, 12));
    const auto result_binary = boost::trim_copy(result_line.substr(5, 12));
    BOOST_CHECK_MESSAGE(trace_binary == result_binary,
                        format("line [%d] : check [binary] : expected [%s] found [%s]") % line_count % result_binary % trace_binary);

    const auto trace_instruction = boost::trim_copy(trace_line.substr(17, 38));
    const auto result_instruction = boost::trim_copy(result_line.substr(17, 38));
    BOOST_CHECK_MESSAGE(trace_instruction == result_instruction,
                        format("line [%d] : check [instruction] : expected [%s] found [%s]") % line_count % result_instruction % trace_instruction);

    const auto trace_details = boost::trim_copy(trace_line.substr(55));
    const auto result_details = boost::trim_copy(result_line.substr(55));

    std::map<std::string, std::string> trace_details_map = build_trace_details_map(trace_details);
    std::map<std::string, std::string> result_details_map = build_trace_details_map(result_details);

    {
      const auto trace_register_a = trace_details_map["A"];
      const auto result_register_a = result_details_map["A"];
      BOOST_CHECK_MESSAGE(trace_register_a == result_register_a,
                          format("line [%d] : check [a] : expected [%s] found [%s]") % line_count % result_register_a % trace_register_a);
    }
    {
      const auto trace_register_x = trace_details_map["X"];
      const auto result_register_x = result_details_map["X"];
      BOOST_CHECK_MESSAGE(trace_register_x == result_register_x,
                          format("line [%d] : check [x] : expected [%s] found [%s]") % line_count % result_register_x % trace_register_x);
    }
    {
      const auto trace_register_y = trace_details_map["Y"];
      const auto result_register_y = result_details_map["Y"];
      BOOST_CHECK_MESSAGE(trace_register_y == result_register_y,
                          format("line [%d] : check [y] : expected [%s] found [%s]") % line_count % result_register_y % trace_register_y);
    }
    {
      const auto trace_register_p = trace_details_map["P"];
      const auto result_register_p = result_details_map["P"];
      BOOST_CHECK_MESSAGE(trace_register_p == result_register_p,
                          format("line [%d] : check [p] : expected [%s] found [%s]") % line_count % result_register_p % trace_register_p);
    }
    {
      const auto trace_register_sp = trace_details_map["SP"];
      const auto result_register_sp = result_details_map["SP"];
      BOOST_CHECK_MESSAGE(trace_register_sp == result_register_sp,
                          format("line [%d] : check [sp] : expected [%s] found [%s]") % line_count % result_register_sp % trace_register_sp);
    }
    {
      const auto trace_ppu_cycle = trace_details_map["CYC"];
      const auto result_ppu_cycle = result_details_map["CYC"];
      BOOST_CHECK_MESSAGE(trace_ppu_cycle == result_ppu_cycle,
                          format("line [%d] : check [ppu_cycle] : expected [%s] found [%s]") % line_count % result_ppu_cycle % trace_ppu_cycle);
    }
    {
      const auto trace_ppu_scanline = trace_details_map["SL"];
      const auto result_ppu_scanline = result_details_map["SL"] == "-1" ? "261" : result_details_map["SL"];
      BOOST_CHECK_MESSAGE(trace_ppu_scanline == result_ppu_scanline,
                          format("line [%d] : check [ppu_scanline] : expected [%s] found [%s]") % line_count % result_ppu_scanline % trace_ppu_scanline);
    }
    {
      const auto trace_ppu_frame = trace_details_map["FC"];
      const auto result_ppu_frame = result_details_map["FC"];
      BOOST_CHECK_MESSAGE(trace_ppu_frame == result_ppu_frame,
                          format("line [%d] : check [ppu_frame] : expected [%s] found [%s]") % line_count % result_ppu_frame % trace_ppu_frame);
    }
    {
      const auto trace_cpu_cycle = trace_details_map["Cycle"];
      const auto result_cpu_cycle = result_details_map["Cycle"];
      BOOST_CHECK_MESSAGE(trace_cpu_cycle == result_cpu_cycle,
                          format("line [%d] : check [cpu_cycle] : expected [%s] found [%s]") % line_count % result_cpu_cycle % trace_cpu_cycle);
    }
    stop_test_if_failed();
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

  trace_file << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << nes_state.cpu.registers.PC << " ";

  for (const auto &i : nes_state.instruction.instruction_bytes) {
    trace_file << "$" << std::right << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<uint16_t>(i) << " ";
  }

  for (size_t i = 0; i < (3 - nes_state.instruction.instruction_bytes.size()); i++) {
    trace_file << " ";
  }

  const size_t instruction_bytes_padding = 9 - (nes_state.instruction.instruction_bytes.size() * 3);
  for (size_t i = 0; i < instruction_bytes_padding; i++) {
    trace_file << " ";
  }

  std::ostringstream instruction_string;

  instruction_string << nes_state.instruction.instruction;

  const auto &instruction_memory = nes_state.instruction.memory;

  if (instruction_memory.is_indirect && instruction_memory.is_indirect.value()) {
    const auto value = nes_state.instruction.memory.address.value();
    const auto fill_width = (value & 0xFF00U) == 0 ? 2 : 4;
    instruction_string << " @ $" << std::hex << std::uppercase << std::setw(fill_width) << std::setfill('0') << value;
  }

  if (instruction_memory.value) {
    instruction_string << " = $" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << nes_state.instruction.memory.value.value();
  }

  trace_file << instruction_string.str();

  const size_t instruction_padding = 38 - instruction_string.str().size();
  for (size_t i = 0; i < instruction_padding; i++) {
    trace_file << " ";
  }

  trace_file << "A:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.A) << " ";
  trace_file << "X:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.X) << " ";
  trace_file << "Y:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.Y) << " ";
  trace_file << "P:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.SR) << " ";
  trace_file << "SP:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.SP) << " ";

  trace_file << "CYC:";

  trace_file << std::dec << std::left << std::setw(3) << std::setfill(' ') << nes_state.ppu.cycle << " ";

  trace_file << "SL:";

  trace_file << std::dec << std::left << std::setw(3) << std::setfill(' ') << nes_state.ppu.scanline << " ";

  trace_file << "FC:";

  trace_file << std::dec << std::left << nes_state.ppu.frame << " ";

  trace_file << "CPU Cycle:" << std::dec << nes_state.cpu.cycle;

  trace_file << std::endl;
}

class trace_listener : public jones::nes_listener {
public:
  explicit trace_listener(std::string trace_path, const int initial_pc)
      : trace_path_(std::move(trace_path)), initial_pc_(initial_pc) {}

  ~trace_listener() override = default;

  auto on_event(jones::nes_listener::event event, jones::nes_state &state) -> void override {
    switch (event) {
    case event::ON_RESET:
    case event::ON_RUN_FINISHED:
    case event::ON_RUN_PAUSED:
    case event::ON_POWER_OFF: {
      //
      // do nothing.
      //
      break;
    }
    case event::ON_POWER_ON: {
      if (initial_pc_ >= 0) {
        state.cpu.registers.PC = initial_pc_;
      }
      break;
    }
    default: {
      add_trace_entry(trace_path_, state);
      break;
    }
    }
  }

private:
  std::string trace_path_;

  const int initial_pc_{};
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
  if (!fs::exists(root_trace_path)) {
    BOOST_FAIL("root trace path is not valid");
    return;
  }
  const auto &root_trace_iterator = fs::directory_iterator(root_trace_path);
  for (auto &trace_directory_entry : boost::make_iterator_range(root_trace_iterator, {})) {
    const auto trace_directory_path = fs::path(trace_directory_entry);
    if (!is_directory(trace_directory_path)) {
      LOG_DEBUG << "skipping trace test [" << trace_directory_path.string() << "]";
      continue;
    }
    const auto trace_file_path = trace_directory_path / fs::path("trace.json");
    if (!fs::exists(trace_file_path)) {
      LOG_DEBUG << "skipping trace test [" << trace_directory_path.string() << "]; test does not exist";
      continue;
    }
    pt::ptree json;
    pt::read_json(trace_file_path.string(), json);

    const auto is_trace_excluded = json.get<bool>("exclude");
    if (is_trace_excluded) {
      LOG_DEBUG << "skipping trace test [" << trace_directory_path.string() << "]; test is excluded";
      continue;
    }
    const auto rom_path = trace_directory_path / json.get<std::string>("rom");
    if (!fs::exists(rom_path)) {
      LOG_DEBUG << "skipping trace test [" << trace_directory_path.string() << "]; rom does not exist";
      continue;
    }
    jones::nes nes(rom_path.string().c_str());
    jones::debugger debugger(nes);

    const auto trace_test_path = fs::temp_directory_path() / fs::unique_path();
    const auto trace_result_path = fs::temp_directory_path() / fs::unique_path();
    const auto trace_name = trace_directory_path.filename().string() + ".trace";

    const auto is_trace_file_built = build_trace_file(trace_result_path, trace_directory_path, trace_name);
    if (is_trace_file_built) {
      const auto &json_initial_state_tree = json.get_child_optional("initial_state");
      const auto initial_pc = json_initial_state_tree ? json_initial_state_tree.value().get<int>("pc") : -1;

      const auto nes_listener = std::make_unique<trace_listener>(trace_test_path.string(), initial_pc);
      debugger.set_listener(nes_listener.get());

      const auto steps_count = json.get_optional<int>("steps");
      const auto line_count = get_line_count(trace_result_path.string());
      const auto run_count = steps_count ? steps_count.value() : line_count;

      if (nes.power()) {
        nes.run(run_count);
        check_trace_files(trace_test_path.string(), trace_result_path.string());
        continue;
      }
    }
    BOOST_FAIL("unable to complete test");
  }
}
