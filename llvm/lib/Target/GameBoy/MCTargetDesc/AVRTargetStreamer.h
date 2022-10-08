//===-- GameBoyTargetStreamer.h - GameBoy Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_GameBoy_TARGET_STREAMER_H
#define LLVM_GameBoy_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic GameBoy target output stream.
class GameBoyTargetStreamer : public MCTargetStreamer {
public:
  explicit GameBoyTargetStreamer(MCStreamer &S);
};

/// A target streamer for textual GameBoy assembly code.
class GameBoyTargetAsmStreamer : public GameBoyTargetStreamer {
public:
  explicit GameBoyTargetAsmStreamer(MCStreamer &S);
};

} // end namespace llvm

#endif // LLVM_GameBoy_TARGET_STREAMER_H
