//===-- GameBoyTargetStreamer.cpp - GameBoy Target Streamer Methods ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides GameBoy specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "GameBoyTargetStreamer.h"

#include "llvm/MC/MCContext.h"

namespace llvm {

GameBoyTargetStreamer::GameBoyTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

GameBoyTargetAsmStreamer::GameBoyTargetAsmStreamer(MCStreamer &S)
    : GameBoyTargetStreamer(S) {}

} // end namespace llvm
