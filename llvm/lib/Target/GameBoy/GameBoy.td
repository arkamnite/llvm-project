//===-- GameBoy.td - Describe the GameBoy Target Machine ----------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
// This is the top level entry point for the GameBoy target.
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===---------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===---------------------------------------------------------------------===//
// GameBoy Device Definitions
//===---------------------------------------------------------------------===//

include "GameBoyDevices.td"

//===---------------------------------------------------------------------===//
// Register File Description
//===---------------------------------------------------------------------===//

include "GameBoyRegisterInfo.td"

//===---------------------------------------------------------------------===//
// Instruction Descriptions
//===---------------------------------------------------------------------===//

include "GameBoyInstrInfo.td"

def GameBoyInstrInfo : InstrInfo;

<<<<<<< HEAD
//===---------------------------------------------------------------------===//
// Calling Conventions
//===---------------------------------------------------------------------===//

include "GameBoyCallingConv.td"

//===---------------------------------------------------------------------===//
// Assembly Printers
//===---------------------------------------------------------------------===//

def GameBoyAsmWriter : AsmWriter {
  string AsmWriterClassName = "InstPrinter";
  bit isMCAsmWriter = 1;

//===---------------------------------------------------------------------===//
// Assembly Parsers
//===---------------------------------------------------------------------===//

def GameBoyAsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 1;
  let ShouldEmitMatchRegisterAltName = 1;
}

def GameBoyAsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
  string TokenizingCharacters = "+";
}

//===---------------------------------------------------------------------===//
// Target Declaration
//===---------------------------------------------------------------------===//

def GameBoy : Target {
  let InstructionSet = GameBoyInstrInfo;
  let AssemblyWriters = [GameBoyAsmWriter];

  let AssemblyParsers = [GameBoyAsmParser];
  let AssemblyParserVariants = [GameBoyAsmParserVariant];
}
=======
def GameBoyAsmParser : AsmParser {
    bit ShouldEmitMatchingRegisterName = 0;
}

>>>>>>> 647717d9b759 (Initial commit)
