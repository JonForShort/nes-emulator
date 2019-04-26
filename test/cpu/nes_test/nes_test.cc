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

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>

#include "cartridge.hh"
#include "log.hh"
#include "nes.hh"

namespace bt = boost::unit_test;
namespace fs = boost::filesystem;

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
  nes.load(file_path);
  nes.trace(trace_path.string().c_str());
  nes.run(10);
}
