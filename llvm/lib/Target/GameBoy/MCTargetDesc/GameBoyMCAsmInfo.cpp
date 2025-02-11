//===-- GameBoyMCAsmInfo.cpp - GameBoy asm properties -----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the GameBoyMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "GameBoyMCAsmInfo.h"

#include "llvm/ADT/Triple.h"

namespace llvm {

GameBoyMCAsmInfo::GameBoyMCAsmInfo(const Triple &TT, const MCTargetOptions &Options) {
  CodePointerSize = 2;
  CalleeSaveStackSlotSize = 2;
  CommentString = ";";
  PrivateGlobalPrefix = ".L";
  PrivateLabelPrefix = ".L";
  UsesELFSectionDirectiveForBSS = false;
  SupportsDebugInformation = true;

  // Remove the .file directive
  HasSingleParameterDotFile = false;

  // RGBASM does not allow multiple instructions on the same line.
  SeparatorString = "\n";

  // RGBASM uses the ".ds" directive similarly to .space in GNU.
  ZeroDirective = "ds ";

  // Data directives in RGBASM are limited to 8-bit (a single byte)
  // only. It is possible to emit multiple bytes in series, such as
  // to emit a number of bytes at a time.
  Data8bitsDirective = "db ";
  Data16bitsDirective = "dw ";
  Data32bitsDirective = "dl ";

  // Do not include the .size directive at the end of the function.
  HasDotTypeDotSizeDirective = false;
}

} // end of namespace llvm
