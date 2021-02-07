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
#define BOOST_TEST_MODULE test_suite_nes_test

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/test/results_collector.hpp>
#include <boost/test/unit_test.hpp>
#include <iomanip>
#include <vector>

#include "cartridge.hh"
#include "debugger.hh"
#include "log.hh"
#include "nes.hh"

namespace bt = boost::unit_test;
namespace fs = boost::filesystem;

using format = boost::format;

namespace {

std::string fix_instruction(std::string instruction) {
  std::vector<std::string> instruction_pieces;
  boost::split(instruction_pieces, instruction, boost::is_any_of("=@"));
  auto const fixed_unofficial = boost::replace_all_copy(instruction_pieces[0], "*", "");
  auto const fixed_opcodes = boost::replace_all_copy(fixed_unofficial, "ISB", "ISC");
  return fixed_opcodes;
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

void check_trace_files(const std::string &trace_path, const std::string &result_path, const size_t expected_line_count) {
  std::ifstream trace_file{trace_path};
  std::ifstream result_file{result_path};
  size_t line_count = 0;
  for (std::string trace_line, result_line;
       std::getline(trace_file, trace_line) && std::getline(result_file, result_line);
       line_count++) {

    auto const trace_address = boost::trim_copy(trace_line.substr(0, 4));
    auto const result_address = boost::trim_copy(result_line.substr(0, 4));
    BOOST_CHECK_MESSAGE(trace_address == result_address,
                        format("line [%d] : check [address] : expected [%s] found [%s]") % line_count % result_address % trace_address);

    auto const trace_binary = boost::trim_copy(trace_line.substr(6, 10));
    auto const result_binary = boost::trim_copy(boost::replace_all_copy(result_line.substr(6, 10), "*", ""));
    BOOST_CHECK_MESSAGE(trace_binary == result_binary,
                        format("line [%d] : check [binary] : expected [%s] found [%s]") % line_count % result_binary % trace_binary);

    auto const trace_instruction = boost::trim_copy(trace_line.substr(15, 16));
    auto const result_instruction = boost::trim_copy(fix_instruction(result_line.substr(15, 16)));
    BOOST_CHECK_MESSAGE(trace_instruction == result_instruction,
                        format("line [%d] : check [instruction] : expected [%s] found [%s]") % line_count % result_instruction % trace_instruction);

    const auto trace_details = boost::trim_copy(trace_line.substr(40));
    const auto result_details = boost::trim_copy(result_line.substr(40));

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
    // {
    //   const auto trace_ppu_cycle = trace_details_map["PPU"];
    //   const auto result_ppu_cycle = result_details_map["PPU"];
    //   BOOST_CHECK_MESSAGE(trace_ppu_cycle == result_ppu_cycle,
    //                       format("line [%d] : check [ppu_cycle] : expected [%s] found [%s]") % line_count % result_ppu_cycle % trace_ppu_cycle);
    // }
    {
      const auto trace_ppu_cycle = trace_details_map["CYC"];
      const auto result_ppu_cycle = result_details_map["CYC"];
      BOOST_CHECK_MESSAGE(trace_ppu_cycle == result_ppu_cycle,
                          format("line [%d] : check [cpu_cycle] : expected [%s] found [%s]") % line_count % result_ppu_cycle % trace_ppu_cycle);
    }
    stop_test_if_failed();
  }

  BOOST_CHECK_MESSAGE(expected_line_count == line_count,
                      format("expected line count does not match actual line count, expected=%d, actual=%d") % expected_line_count % line_count);
}

class trace_listener : public jones::nes_listener {
public:
  explicit trace_listener(std::string trace_path) : trace_path_(std::move(trace_path)) {}

  ~trace_listener() override = default;

  auto on_event(jones::nes_listener::event event, jones::nes_state &state) -> void override {
    switch (event) {
    case event::ON_RESET:
    case event::ON_INSTRUCTION_FINISHED:
    case event::ON_RUN_STARTED:
    case event::ON_RUN_FINISHED:
    case event::ON_RUN_PAUSED:
    case event::ON_RUN_STEP:
    case event::ON_POWER_OFF: {
      //
      // do nothing.
      //
      break;
    }
    case event::ON_POWER_ON: {
      state.cpu.registers.PC = 0xC000;
      state.cpu.registers.SR = 0x24;
      state.cpu.cycles.running = 7;
      break;
    }
    case event::ON_INSTRUCTION_STARTED: {
      add_trace_entry(trace_path_, state);
      break;
    }
    default: {
      BOOST_STATIC_ASSERT("missing listener event");
      break;
    }
    }
  }

private:
  void add_trace_entry(const fs::path &trace_path, const jones::nes_state &nes_state) {
    std::ofstream trace_file(trace_path, std::ios::app);

    trace_file << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << nes_state.cpu.registers.PC << "  ";

    for (const auto &i : nes_state.instruction.instruction_bytes) {
      trace_file << std::right << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<uint16_t>(i) << " ";
    }

    const size_t instruction_bytes_padding = 10 - (nes_state.instruction.instruction_bytes.size() * 3);
    for (size_t i = 0; i < instruction_bytes_padding; i++) {
      trace_file << " ";
    }

    trace_file << nes_state.instruction.instruction;

    const size_t instruction_padding = 26 - nes_state.instruction.instruction_bytes.size();
    for (size_t i = 0; i < instruction_padding; i++) {
      trace_file << " ";
    }

    trace_file << "A:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.A) << " ";
    trace_file << "X:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.X) << " ";
    trace_file << "Y:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.Y) << " ";
    trace_file << "P:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.SR) << " ";
    trace_file << "SP:" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(nes_state.cpu.registers.SP) << " ";

    trace_file << "PPU:";

    trace_file << std::dec << std::right << std::setw(3) << std::setfill(' ') << nes_state.ppu.cycle << ",";

    trace_file << std::dec << std::right << std::setw(3) << std::setfill(' ') << nes_state.ppu.scanline << " ";

    trace_file << "CYC:" << std::dec << nes_state.cpu.cycles.running;

    trace_file << std::endl;
  }

  std::string trace_path_;
};

} // namespace

BOOST_AUTO_TEST_CASE(test_suite_nes_test) {
  auto const argc = bt::framework::master_test_suite().argc;
  if (argc <= 2) {
    BOOST_CHECK_MESSAGE(false, "missing required test arguments");
    return;
  }
  auto const argv = bt::framework::master_test_suite().argv;
  auto const file_path = argv[1];
  auto const result_path = argv[2];

  const fs::path trace_path = fs::temp_directory_path() / fs::unique_path();

  LOG_DEBUG << "test arguments : "
            << "binary [" << file_path << "] "
            << "result [" << result_path << "] "
            << "trace [" << trace_path << "]";

  jones::nes nes(file_path);
  jones::debugger debugger(nes);

  auto const nes_listener = std::make_unique<trace_listener>(trace_path.string());
  debugger.set_listener(nes_listener.get());

  if (nes.power()) {
    auto const step_limit = 8992;
    nes.run(step_limit);
    check_trace_files(trace_path.string(), result_path, step_limit);
  }
}
