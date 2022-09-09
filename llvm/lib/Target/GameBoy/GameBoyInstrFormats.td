// This format specifies the encoding used by instructions.
// Used by the code emitter to determine which form of "fixup"
// to use; these are kept in sync with GameBoyBaseInfo.h

class GBInstr8<dag outs, dag ins, string asmstr, list<dag> pattern, InstrItinClass itin = NoItinerary> : Instruction {
    
    // Instructions are 8-bits long.
    field bits<8> Inst;

    let Namespace = "GB";
    // All instructions are 8 bits long.
    let Size = 8;

    dag OutOperandList = outs;
    dag InOperandList = ins;
    let AsmString = asmstr;
    let Pattern = pattern;

    let DecoderNamespace = "GameBoy";
    field bits<8> SoftFail = 0;

    let Itinerary = itin;
}

class GBInstr16<dag outs, dag ins, string asmstr, list<dag> pattern, InstrItinClass itin = NoItinerary> : Instruction {
    let Namespace = "GB";
    
    // Instructions are 16-bits long.
    field bits<16> Inst;
    let Size = 16;

    dag OutOperandList = outs;
    dag InOperandList = ins;
    let AsmString = asmstr;
    let Pattern = pattern;

    let DecoderNamespace = "GameBoy";
}

class GBInstr24<dag outs, dag ins, string asmstr, list<dag> pattern, InstrItinClass itin = NoItinerary> : Instruction {
    let Namespace = "GB";

    // Instructions are 24-bits long, due to two operands.
    field bits<24> Inst;
    let Size = 24;

    dag OutOperandList = outs;
    dag InOperandList = ins;
    dag AsmString = asmstr;
    let Pattern = pattern;

    let DecoderNamespace = "GameBoy"
}

//=== Register codes ===//
// A : 111
// B : 000
// C : 001
// D : 101
// E : 011
// H : 100
// L : 101


//===---------------------------------------------------------------===//
// GameBoy instruction formats are derived from the types of operands
// each instruction takes.
//===---------------------------------------------------------------===//

// Format 1: No operand
// xx - xxx - xxx
// class F1<   dag outs, dag ins, string asmstr, list<dag> pattern, 
//             InstrItinClass itin = NoItinerary> : GBInstr8<outs, in, asmstr, pattern, itin> {
    
// }

// Cycle operand instruction
// xx - xCC - xxx

// Format 2: Register operand instruction
// xx - rrr - xxx
class F2< dag outs, dag ins, string asmstr, list<dag> pattern,
    InstrItinClass itin = NoItinerary> : GBInstr8<outs, in, asmstr, pattern, itin> 
{
    bits<3>         r;
    let op =        0;
    let Inst{5-3} = r;
}

// Register operand instruction 2 (arithmetic and logic instructions)
// xx - xxx - rrr

// Register pair operand instruction
// xx - rrx - xxx

// Register-register operand instruction
// xx - rrr - rrr

// Immediate 8-bit data instruction
// xx - xxx - xxx
// n8

// Immediate 8-bit data, with register, instruction
// xx - rrr - xxx
// n8

// Immediate 16-bit data instruction
// xx - xxx - xxx
// n8
// n8

// Immediate 16-bit data instruction, with register pair
// xx - ddx - xxx
// n8
// n8

// Immediate 16-bit instruction, with cycle
// xx - xcc - xxx
// n8
// n8

//===---------------------------------------------------------------===//
// Rotate shift instructions follow a new type of format
//===---------------------------------------------------------------===//

// Rotate register
// xx - xxx - xxx
// xx - xxx - rrr

// Rotate register pair
// xx - xxx - xxx
// xx - xxx - xxx

//===---------------------------------------------------------------===//
// Bit operation instructions also follow a different format.
//===---------------------------------------------------------------===//

// Bit-register instruction
// xx - xxx - xxx
// xx - bbb - rrr

// Bit instruction
// xx - xxx - xxx
// xx - bbb - xxx

