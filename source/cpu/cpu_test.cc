//
// MIT License
//
// Copyright 2018
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
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include "cpu.h"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace ip = boost::interprocess;
namespace jo = jones;

int main(int argc, char *argv[]) {

  std::string fileArgument;
  bool disassembleArgument;

  try {
    po::options_description desc{"Options"};
    desc.add_options()
      ("help,h", "Help screen")
      ("file,f", po::value<std::string>(&fileArgument)->required(), "path to binary file")
      ("disassemble,d", po::bool_switch(&disassembleArgument), "perform disassemble operation");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const po::error &ex) {
    std::cerr << ex.what() << '\n';
    return -1;
  }

  const fs::path pathToBinaryFile(fileArgument);
  if (!fs::exists(pathToBinaryFile)) {
    std::cout << "binary file path does not exist; please check path" << std::endl;
    return -2;
  }

  if (fs::is_directory(pathToBinaryFile)) {
    std::cout << "binary file path is a directory; please check path" << std::endl;
    return -3;
  }

  const auto performDisassemble = disassembleArgument;
  if (performDisassemble) {
    std::cout << "performing disassemble operation" << std::endl;
    ip::file_mapping mappedBinary(pathToBinaryFile.c_str(), ip::mode_t::read_only);
    ip::mapped_region mappedRegion(mappedBinary, ip::mode_t::read_only);
    const auto mappedStartAddress = mappedRegion.get_address();
    const auto mappedSize = mappedRegion.get_size();

    jo::Cpu cpu(mappedStartAddress);;
    cpu.run();
  }

  return 0;
}
