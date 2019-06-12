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
#include <boost/test/unit_test.hpp>
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

} // namespace

BOOST_AUTO_TEST_CASE(test_suite_nes_test) {
  const auto argc = bt::framework::master_test_suite().argc;
  if (argc <= 2) {
    BOOST_CHECK_MESSAGE(false, "missing required test arguments");
    return;
  }
  const auto argv = bt::framework::master_test_suite().argv;
  const auto file_path = argv[1];
  const auto result_path = argv[2];

  const fs::path trace_path = fs::temp_directory_path() / fs::unique_path();

  LOG_DEBUG << "test arguments : "
            << "binary [" << file_path << "] "
            << "result [" << result_path << "] "
            << "trace [" << trace_path << "]";

  jones::nes nes;

  jones::debugger debugger(nes);
  debugger.trace(trace_path.string().c_str());

  nes.load(file_path);
  nes.run(8992);

  check_trace_files(trace_path.string(), result_path);
}
