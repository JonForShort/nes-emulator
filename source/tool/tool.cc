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
#include <fstream>
#include <iostream>
#include <string>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char *argv[]) {

  std::string fileArgument;
  std::string outputArgument;

  try {
    po::options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")("file,f", po::value<std::string>(&fileArgument)->required(), "file path to rom")("output,o", po::value<std::string>(&outputArgument)->default_value("out"), "directory path to write output");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const po::error &ex) {
    std::cerr << ex.what() << '\n';
    return -1;
  }

  const fs::path filePath(fileArgument);
  if (!fs::exists(filePath)) {
    std::cerr << "file path does not exist; please check path" << std::endl;
    return -2;
  }

  if (fs::is_directory(filePath)) {
    std::cerr << "file path is a directory; please check path" << std::endl;
    return -3;
  }

  const fs::path outputPath(outputArgument);
  if (fs::exists(outputPath) && !fs::is_directory(outputPath)) {
    std::cerr << "output directory path is a file; please check path : " << outputPath << std::endl;
    return -4;
  }

  if (!fs::exists(outputPath) && !fs::create_directories(outputPath)) {
    std::cerr << "unable to create output directory path; please check path : " << outputPath << std::endl;
    return -5;
  }

  return 0;
}
