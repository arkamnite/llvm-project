// GameBoyRegisterInfo.td - Game Boy Register definitions

class GBCtrlReg<string n> : Register<n> {
    let Namespace = "GameBoy";
}

let Namespace = "GameBoy" in {
def sub_low : SubRegIndex<8>;
def sub_high : SubRegIndex<8, 8>;
}


class Reg<bits<16> number, 
          string name, 
          list<Register> subregs = [],
          list<string> altNames = []> : RegisterWithSubRegs<name, subregs> {
    
    field bits<16> Num = number;
    let HWEncoding = number;
    let Namespace = "GameBoy";
    let SubRegs = subregs;
    let AltNames = altNames;
}

// A pair of registers
class RegPair<bits<16> number, 
              string n, 
              list<Register> subregs,
              list<string> altNames = []> : Reg<number, n, subregs, altNames> {
    let Namespace = "GameBoy";
    let SubRegs = subregs;
    let SubRegIndices = [sub_high, sub_low];
}

// Control register.
def Flags : GBCtrlReg<"Flags">;

// Individual 8-bit registers.
def A : Reg<0, "A">, DwarfRegNum<[0]>;
def B : Reg<1, "B">, DwarfRegNum<[1]>;
def C : Reg<2, "C">, DwarfRegNum<[2]>;
def D : Reg<3, "D">, DwarfRegNum<[3]>;
def E : Reg<4, "E">, DwarfRegNum<[4]>;
def H : Reg<5, "H">, DwarfRegNum<[5]>;
def L : Reg<6, "L">, DwarfRegNum<[6]>;
def F : Reg<7, "F">, DwarfRegNum<[7]>;

// Todo: Add PC and SP

// Register Pair aliases
let SubRegIndices = [sub_low, sub_high], CoveredBySubRegs = 1 in {
    def BC : RegPair<1, "BC", [B, C]>, DwarfRegNum<[10]>;
    def DE : RegPair<3, "DE", [D, E]>, DwarfRegNum<[11]>;
    def HL : RegPair<5, "HL", [H, L]>, DwarfRegNum<[12]>;
}

// Register classes
// There is only one type of general use register
def I8Regs : RegisterClass<"GameBoy", [i8], 8, (add A, B, C, D, E, H, L)>;
def I8Pair : RegisterClass<"GameBoy", [v2i8], 16, (add BC, DE, HL)>;