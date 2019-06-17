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
#include <memory>

#include "mapper.hh"
#include "mapper_mmc1.hh"
#include "mapper_nrom.hh"
#include "mapper_unsupported.hh"

using namespace jones;

std::unique_ptr<mapper> mapper_factory::get(const mapper_view &mapper_view) {
  const auto mapper_number = mapper_view.cartridge().header()->mapper_number();
  switch (mapper_number) {
  case 0:
    return std::make_unique<mapper_nrom>(mapper_view);
  case 1:
    return std::make_unique<mapper_mmc1>(mapper_view);
  default:
    return std::make_unique<mapper_unsupported>(mapper_view);
  }
}
