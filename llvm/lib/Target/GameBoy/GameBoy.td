// GameBoy tablegen file

include "llvm/Target/Target.td"

// GameBoy subtarget features

// DMG Register File Description

include "GameBoyRegisterInfo.td"

// DMG Instruction Descriptions

include "GameBoyInstrInfo.td"
def GameBoyInstrInfo : InstrInfo;

// DMG Calling Conventions

// include "GameBoyCallingConv.td"

// Assembly Printers

def GameBoyAsmWriter: AsmWriter {
    string AsmWriterClassName = "InstPrinter";
    bit isMCAsmWriter = 1;
}

// Assembly Parsers

def GameBoyAsmParser : AsmParser {
    bit ShouldEmitMatchingRegisterName = 0;
}

def GameBoyAsmParserVariant : AsmParserVariant {
    int Variant = 0;
}

// Target

def GameBoy : Target {
    let InstructionSet = GameBoyInstrInfo;
    let AssemblyParsers = [GameBoyAsmParser];
    let AssemblyWriters = [GameBoyAsmWriter];
}