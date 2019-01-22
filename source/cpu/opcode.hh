//
// MIT License
//
// Copyright 2017-2018
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
#pragma once

//
// CPU reference
// http://e-tradition.net/bytes/6502/6502_instruction_set.html
// https://wiki.nesdev.com/w/index.php/Programming_with_unofficial_opcodes
//
namespace jones {

enum class opcode_type {

  // opcode is unknown or invalid.
  INVALID,

  // ADD with carry
  ADC,

  // STORE A X H into address (unofficial)
  AHX,

  // AND immediate then LSR A (unofficial)
  ALR,

  // AND immediate value then copies N flag to C (unofficial)
  ANC,

  // AND (with accumulator)
  AND,

  // AND immediate then ROR A (unofficial)
  ARR,

  // arithmetic shift left
  ASL,

  // set X to A AND X and update NZC (unofficial)
  AXS,

  // branch on carry clear
  BCC,

  // branch on carry set
  BCS,

  // branch on equal (zero set)
  BEQ,

  // bit test
  BIT,

  // branch on minus (negative set)
  BMI,

  // branch on not equal (zero clear)
  BNE,

  // branch on plus (negative clear)
  BPL,

  // interrupt
  BRK,

  // branch on overflow clear
  BVC,

  // branch on overflow set
  BVS,

  // clear carry
  CLC,

  // clear decimal
  CLD,

  // clear interrupt disable
  CLI,

  // clear overflow
  CLV,

  // compare (with accumulator)
  CMP,

  // compare with X
  CPX,

  // compare with Y
  CPY,

  // DEC value then CMP value (unofficial)
  DCP,

  // decrement
  DEC,

  // decrement X
  DEX,

  // decrement Y
  DEY,

  // exclusive or (with accumulator)
  EOR,

  // increment
  INC,

  // increment x
  INX,

  // increment y
  INY,

  // INC value then SBC value (unofficial)
  ISC,

  // jump
  JMP,

  // jump subroutine
  JSR,

  // store address AND S into A X and S (unofficial)
  LAS,

  // LDA value then TAX value (unofficial)
  LAX,

  // load accumulator
  LDA,

  // load x
  LDX,

  // load y
  LDY,

  // logical shift right
  LSR,

  // no operation
  NOP,

  // or with accumulator
  ORA,

  // push accumulator
  PHA,

  // push processor status (SR)
  PHP,

  // pull accumulator
  PLA,

  // pull processor status (SR)
  PLP,

  // ROL followed by AND
  RLA,

  // rotate left
  ROL,

  // rotate right
  ROR,

  // ROR and then ADC (unofficial)
  RRA,

  // return from interrupt
  RTI,

  // return from subroutine
  RTS,

  // stores bitwise AND of A and X (unofficial)
  SAX,

  // subtract with carry
  SBC,

  // set carry
  SEC,

  // set decimal
  SED,

  // set interrupt disable
  SEI,

  // stores H AND X into address (unofficial)
  SHX,

  // stores H and Y into address (unofficial)
  SHY,

  // shift left
  SLO,

  // LSR value then EOR value (unofficial)
  SRE,

  // store accumulator
  STA,

  // store X
  STX,

  // store Y
  STY,

  // store P
  STP,

  // store A AND X into S and A AND X AND H into address (unofficial)
  TAS,

  // transfer accumulator to X
  TAX,

  // transfer accumulator to Y
  TAY,

  // transfer stack pointer to X
  TSX,

  // transfer stack pointer to Y
  TSY,

  // transfer X to accumulator
  TXA,

  // transfer X to stack pointer
  TXS,

  // transfer Y to accumulator
  TYA,

  // transfer Y to stack pointer
  TYS,

  // unknown (unofficial)
  XAA
};

} // namespace jones
