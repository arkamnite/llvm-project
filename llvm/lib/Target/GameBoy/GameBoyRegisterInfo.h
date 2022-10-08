#ifndef LLVM_GAMEBOY_REGISTER_INFO_H
#define LLVM_GAMEBOY_REGISTER_INFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "GameBoyRegisterInfo.inc"

namespace llvm {

class GameBoyRegisterInfo : public GameBoyGenRegisterInfo {
public:
    GameBoyRegisterInfo();

public:
    // Function stubs live here.
    const uint16_t *getCalleeSavedRegs(const MachineFunction *MF = nullptr) const override;
    
    BitVector getReservedRegs(const MachineFunction &MF) const  override;
    
    /// Stack Frame Processing Methods
    void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj, 
                             unsigned FIOperandNum, RegScavenger *RS = nullptr) const override;

    Register getFrameRegister(const MachineFunction &MF) const override;
    /*
    const uint32_t *getCallPreservedMask(const MachineFunction &MF, CallingConv::ID CC) const override;

    const TargetRegisterClass *getLargestLegalSuperClass(const TargetRegisterClass *RC, const MachineFunction &MF) const override;


    const TargetRegisterClass *getPointerRegClass(const MachineFunction &MF, unsigned Kind = 0) const override;

    /// Splits a 16-bit register pair into the low-high register pair.
    /// \param Reg A 16-bit register to split.
    void splitReg(Register Reg, Register &LoReg, Register &HiReg) const;

    bool shouldCoalesce(MachineInstr *MI, 
                        const TargetRegisterClass *SrcRC, 
                        unsigned SubReg, const TargetRegisterClass *DstRC, 
                        unsigned DstSubReg, const TargetRegisterClass *NewRC, 
                        LiveIntervals &LIS) const override;
    */
}; 

} // end namespace llvm

#endif // LLVM_GAMEBOY_REGISTER_INFO_H