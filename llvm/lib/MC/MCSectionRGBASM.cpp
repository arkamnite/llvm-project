#include "llvm/MC/MCSectionRGBASM.h"
#include "llvm/ADT/Triple.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <iostream>
#include <cassert>

using namespace llvm;

