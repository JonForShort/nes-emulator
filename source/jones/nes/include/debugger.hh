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
#ifndef JONES_NES_DEBUGGER_HH
#define JONES_NES_DEBUGGER_HH

#include <cstdint>
#include <memory>

namespace jones {

class nes;

struct nes_state {

  struct cpu {

    size_t cycle;

    struct registers {

      uint16_t PC;

      uint8_t A;

      uint8_t X;

      uint8_t Y;

      uint8_t SR;

      uint8_t SP;

    } registers;

  } cpu;

  struct ppu {

    size_t cycle;

    size_t scanline;

    size_t frame;

  } ppu;

  struct apu {

    size_t cycle;

  } apu;

  struct instruction {

    std::string instruction;

    std::vector<uint8_t> instruction_bytes;

    struct memory {

      std::optional<uint16_t> address;

      std::optional<uint16_t> value;

      std::optional<bool> is_indirect;

    } memory;

  } instruction;
};

class nes_listener {
public:
  enum class event {
    ON_RUN_STARTED,

    ON_RUN_PAUSED,

    ON_RUN_FINISHED,

    ON_RUN_STEP,

    ON_INSTRUCTION_STARTED,

    ON_INSTRUCTION_FINISHED,

    ON_RESET,

    ON_POWER_ON,

    ON_POWER_OFF,
  };

  virtual ~nes_listener() = default;

  virtual auto on_event(event event, nes_state &state) -> void = 0;
};

class debugger final {
public:
  explicit debugger(const nes &nes);

  ~debugger();

  auto read(uint16_t address) -> uint8_t;

  auto write(uint16_t address, uint8_t data) -> void;

  auto set_listener(nes_listener *listener) -> void;

private:
  class impl;

  std::unique_ptr<impl> impl_;
};

} // namespace jones

#endif // JONES_NES_DEBUGGER_HH
