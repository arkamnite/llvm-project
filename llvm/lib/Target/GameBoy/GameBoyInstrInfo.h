#ifndef LLVM_GAMEBOY_INSTR_INFO_H
#define LLVM_GAMEBOY_INSTR_INFO_H

#include "llvm/CodeGen/TargetInstrInfo.h"

#include "GameBoyRegisterInfo.h"

#define GET_INSTRINFO_HEADER
#include "GameBoyInstrInfo.inc"
#undef GET_INSTRINFO_HEADER

namespace llvm {

namespace GameBoyCC {

} // end namespace GameBoyCC

class GameBoyInstrInfo : public GameBoyGenInstrInfo {
public:
    explicit GameBoyInstrInfo();

    const GameBoyRegisterInfo &getRegisterInfo() const { return RI; }

    /// Copied from AVRInstrInfo.h
    void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                   const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg,
                   bool KillSrc) const override;
    void storeRegToStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MI, Register SrcReg,
                            bool isKill, int FrameIndex,
                            const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI) const override;
    void loadRegFromStackSlot(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI, Register DestReg,
                                int FrameIndex, const TargetRegisterClass *RC,
                                const TargetRegisterInfo *TRI) const override;
    unsigned isLoadFromStackSlot(const MachineInstr &MI,
                                int &FrameIndex) const override;
    unsigned isStoreToStackSlot(const MachineInstr &MI,
                                int &FrameIndex) const override;

    // Branch analysis.
    bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                        MachineBasicBlock *&FBB,
                        SmallVectorImpl<MachineOperand> &Cond,
                        bool AllowModify = false) const override;
    unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                            MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                            const DebugLoc &DL,
                            int *BytesAdded = nullptr) const override;
    unsigned removeBranch(MachineBasicBlock &MBB,
                            int *BytesRemoved = nullptr) const override;
    bool
    reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const override;

    MachineBasicBlock *getBranchDestBlock(const MachineInstr &MI) const override;

    bool isBranchOffsetInRange(unsigned BranchOpc,
                                int64_t BrOffset) const override;

    void insertIndirectBranch(MachineBasicBlock &MBB,
                                MachineBasicBlock &NewDestBB,
                                MachineBasicBlock &RestoreBB, const DebugLoc &DL,
                                int64_t BrOffset, RegScavenger *RS) const override;

private:
    const GameBoyRegisterInfo RI;
};

} // end namespace llvm

#endif // LLVM_GAMEBOY_INSTR_INFO_H