#include "GameBoyInstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

#include "GameBoy.h"
// #include "GameBoyMachineFunctionInfo.h"
#include "GameBoyRegisterInfo.h"
#include "GameBoyTargetMachine.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "GameBoyInstrInfo.inc"

namespace llvm {

GameBoyInstrInfo::GameBoyInstrInfo() : GameBoyGenInstrInfo(), RI() {}

} // end namespace llvm