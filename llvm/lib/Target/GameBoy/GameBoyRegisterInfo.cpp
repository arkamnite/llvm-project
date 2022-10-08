#include "GameBoyRegisterInfo.h"

#include "GameBoy.h"
#include "GameBoyTargetMachine.h"
#include "GameBoyInstrInfo.h"
#include "GameBoyMachineFunction.h"

#define GET_REGINFO_TARGET_DESC
#include "GameBoyRegisterInfo.inc"

namespace llvm {

GameBoyRegisterInfo::GameBoyRegisterInfo() : GameBoyGenRegisterInfo(0) {}

const uint16_t *
GameBoyRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
    return CSR_Normal_SaveList;
}

BitVector GameBoyRegisterInfo::getReservedRegs(const MachineFunction *MF) const {
    BitVector Reserved(getNumRegs());

    // TODO: Set the reserved registers here. 
    return Reserved;
}

void GameBoyRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                              int SPAdj, unsigned FIOperandNum,
                                              RegScavenger *RS) const {

    return;
}

} // end namespace llvm