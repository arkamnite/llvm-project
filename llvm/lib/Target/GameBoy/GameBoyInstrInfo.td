include "GameBoyInstrFormats.td"

//=== Operands ===//
// Immediate 8-bit operand, use i8imm

// Immediate 16-bit operand, use i16imm

// Todo: Complete to ensure no defs must be made.

//=== Instructions ===//

def LDRdRr : FRdRr< 0b01, 
                    (outs GPR8: $rd),
                    (ins GPR8: $rr),
                    "ld\t$rd, $rr",
                    [(set i8 : $rd, i8 : $rr)]>;