#include "GameBoyTargetStreamer.h"

#include "llvm/MC/MCContext.h"

namespace llvm {

GameBoyTargetStreamer::GameBoyTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

GameBoyTargetAsmStreamer::GameBoyTargetAsmStreamer(MCStreamer &S)
    : GameBoyTargetStreamer(S) {}

} // end namespace llvm