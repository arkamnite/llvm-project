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