// GameBoy tablegen file

include "llvm/Target/Target.td"

// GameBoy subtarget features

// Register File, Calling Convention, Instruction Descriptions

include "GameBoyRegisterInfo.td"
include "GameBoyCallingConv.td"
include "GameBoySchedule.td"
include "GameBoyInstrInfo.td"

def GameBoyInstrInfo : InstrInfo;

def GameBoyAsmParser : AsmParser {
    bit ShouldEmitMatchingRegisterName = 0;
}

