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
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>

#include "tool.hh"

namespace fs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char *argv[]) {

  std::string file_argument;
  std::string output_argument;

  try {
    po::options_description desc{"Options"};
    desc.add_options()
    ("help,h", "Help screen")
    ("file,f", po::value<std::string>(&file_argument)->required(), "file path to rom")
    ("output,o", po::value<std::string>(&output_argument)->default_value("out"), "directory path to write output");

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

  jones::tool::dump(file_path.string().c_str(), output_path.string().c_str());

  return 0;
}
