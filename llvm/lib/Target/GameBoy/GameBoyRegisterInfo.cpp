#include "GameBoyRegisterInfo.h"

#include "GameBoy.h"
#include "GameBoyTargetMachine.h"
#include "GameBoyInstrInfo.h"
#include "GameBoyMachineFunction.h"

#define GET_REGINFO_TARGET_DESC
#include "GameBoyRegisterInfo.inc"

namespace llvm {

GameBoyRegisterInfo::GameBoyRegisterInfo() : GameBoyGenRegisterInfo(0) {}

} // end namespace llvm