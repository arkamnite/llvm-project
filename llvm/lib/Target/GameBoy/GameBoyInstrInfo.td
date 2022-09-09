include "GameBoyInstrFormats.td"

//=== Operands ===//
// Immediate 8-bit operand, use i8imm

// Immediate 16-bit operand, use i16imm

// Todo: Complete to ensure no defs must be made.

//=== Instructions ===//

def LDri : F2<  (outs I8Regs:$dst), (ins i8imm:$asi),
                "ld $dst, $asi",
                [(set i8:$dst, i8imm:$asi)]>;
