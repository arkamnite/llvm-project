//===- lib/MC/MCSectionELF.cpp - ELF Code Section Representation ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCSectionELF.h"
#include "llvm/ADT/Triple.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <iostream>
#include <cassert>

using namespace llvm;

/// Used to determine if this has a LOAD block which must be terminated.
bool openLoadBlock;

// Decides whether a '.section' directive
// should be printed before the section name.
bool MCSectionELF::shouldOmitSectionDirective(StringRef Name,
                                              const MCAsmInfo &MAI) const {
  if (isUnique())
    return false;

  return MAI.shouldOmitSectionDirective(Name);
}

static void printName(raw_ostream &OS, StringRef Name) {
  if (Name.find_first_not_of("0123456789_."
                             "abcdefghijklmnopqrstuvwxyz"
                             "ABCDEFGHIJKLMNOPQRSTUVWXYZ") == Name.npos) {
    OS << Name;
    return;
  }
  OS << '"';
  for (const char *B = Name.begin(), *E = Name.end(); B < E; ++B) {
    if (*B == '"') // Unquoted "
      OS << "\\\"";
    else if (*B != '\\') // Neither " or backslash
      OS << *B;
    else if (B + 1 == E) // Trailing backslash
      OS << "\\\\";
    else {
      OS << B[0] << B[1]; // Quoted character
      ++B;
    }
  }
  OS << '"';
}

void MCSectionELF::printSwitchToSection(const MCAsmInfo &MAI, const Triple &T,
                                        raw_ostream &OS,
                                        const MCExpr *Subsection) const {
  
  // RGBASM has very specific section descriptions.
  if (T.getArch() == Triple::gameboy) {

    // It isn't possible to create a SECTION in RAM. The only
    // way to do this is to use a LOAD block to move the code
    // and data into the W/RAM banks. This is included in a
    // ROM/ROMX section, so don't emit a new SECTION directive
    // unless necessary.

    // We need to handle printing of various data sections
    // manually. For RGBASM, this means that there are four
    // destinations for data.
    //
    // .text -> ROM0[$100]
    // .data -> WRAM0/WRAMX
    // .rodata -> ROM0/ROMX
    // .bss -> WRAM0/WRAMX
    //
    // This can be implemented via a switch on the name;
    // anything else can go into ROM0, and will be
    // concatenated to the section.

    // std::string rombankStr;
    auto sectionName = getName().str();
    // std::cout << sectionName << std::endl;

    // If we have an open LOAD block, then it must be closed before creating a new section
    if (openLoadBlock) {
      OS << "\tENDL\n\n";
      openLoadBlock = false;
    }

    if (sectionName.compare(".data") == 0 || sectionName.compare(".bss") == 0) {
      // Create a label for this
      OS << sectionName << ":\n";
      // Place the LOAD block on a new line.
      OS << "\tLOAD \"";
      printName(OS, getName());
      OS << "\", ";
      OS << "WRAM0";
      openLoadBlock = true;
      // rombankStr = std::string("WRAM0");
    }
    else {
      OS << "INCLUDE \"dmglib.inc\"\n\n";
      OS << "SECTION \"";
      printName(OS, getName());
      OS << "\", ";
      OS << "ROM0";
    }
      // rombankStr = std::string("ROM0");

    // OS << rombankStr;    
    // Check if this is the .text section, and if so, place it
    // at address [$100] in ROM0
    if (!(sectionName.compare(".text")))
      OS << "[$100]";

    // OS << "\", ROM0[$100]\n";
    OS << "\n";
    return;
  }

  if (shouldOmitSectionDirective(getName(), MAI)) {
    OS << '\t' << getName();
    if (Subsection) {
      OS << '\t';
      Subsection->print(OS, &MAI);
    }
    OS << '\n';
    return;
  }

  // Handle the weird solaris syntax if desired.
  if (MAI.usesSunStyleELFSectionSwitchSyntax() &&
      !(Flags & ELF::SHF_MERGE)) {
    if (Flags & ELF::SHF_ALLOC)
      OS << ",#alloc";
    if (Flags & ELF::SHF_EXECINSTR)
      OS << ",#execinstr";
    if (Flags & ELF::SHF_WRITE)
      OS << ",#write";
    if (Flags & ELF::SHF_EXCLUDE)
      OS << ",#exclude";
    if (Flags & ELF::SHF_TLS)
      OS << ",#tls";
    OS << '\n';
    return;
  }

  OS << ",\"";
  if (Flags & ELF::SHF_ALLOC)
    OS << 'a';
  if (Flags & ELF::SHF_EXCLUDE)
    OS << 'e';
  if (Flags & ELF::SHF_EXECINSTR)
    OS << 'x';
  if (Flags & ELF::SHF_GROUP)
    OS << 'G';
  if (Flags & ELF::SHF_WRITE)
    OS << 'w';
  if (Flags & ELF::SHF_MERGE)
    OS << 'M';
  if (Flags & ELF::SHF_STRINGS)
    OS << 'S';
  if (Flags & ELF::SHF_TLS)
    OS << 'T';
  if (Flags & ELF::SHF_LINK_ORDER)
    OS << 'o';
  if (Flags & ELF::SHF_GNU_RETAIN)
    OS << 'R';

  // If there are os-specific flags, print them.
  if (T.isOSSolaris())
    if (Flags & ELF::SHF_SUNW_NODISCARD)
      OS << 'R';

  // If there are target-specific flags, print them.
  Triple::ArchType Arch = T.getArch();
  
  if (Arch == Triple::xcore) {
    if (Flags & ELF::XCORE_SHF_CP_SECTION)
      OS << 'c';
    if (Flags & ELF::XCORE_SHF_DP_SECTION)
      OS << 'd';
  } else if (T.isARM() || T.isThumb()) {
    if (Flags & ELF::SHF_ARM_PURECODE)
      OS << 'y';
  } else if (Arch == Triple::hexagon) {
    if (Flags & ELF::SHF_HEX_GPREL)
      OS << 's';
  }

  OS << '"';

  OS << ',';

  // If comment string is '@', e.g. as on ARM - use '%' instead
  if (MAI.getCommentString()[0] == '@')
    OS << '%';
  else
    OS << '@';

  if (Type == ELF::SHT_INIT_ARRAY)
    OS << "init_array";
  else if (Type == ELF::SHT_FINI_ARRAY)
    OS << "fini_array";
  else if (Type == ELF::SHT_PREINIT_ARRAY)
    OS << "preinit_array";
  else if (Type == ELF::SHT_NOBITS)
    OS << "nobits";
  else if (Type == ELF::SHT_NOTE)
    OS << "note";
  else if (Type == ELF::SHT_PROGBITS)
    OS << "progbits";
  else if (Type == ELF::SHT_X86_64_UNWIND)
    OS << "unwind";
  else if (Type == ELF::SHT_MIPS_DWARF)
    // Print hex value of the flag while we do not have
    // any standard symbolic representation of the flag.
    OS << "0x7000001e";
  else if (Type == ELF::SHT_LLVM_ODRTAB)
    OS << "llvm_odrtab";
  else if (Type == ELF::SHT_LLVM_LINKER_OPTIONS)
    OS << "llvm_linker_options";
  else if (Type == ELF::SHT_LLVM_CALL_GRAPH_PROFILE)
    OS << "llvm_call_graph_profile";
  else if (Type == ELF::SHT_LLVM_DEPENDENT_LIBRARIES)
    OS << "llvm_dependent_libraries";
  else if (Type == ELF::SHT_LLVM_SYMPART)
    OS << "llvm_sympart";
  else if (Type == ELF::SHT_LLVM_BB_ADDR_MAP)
    OS << "llvm_bb_addr_map";
  else if (Type == ELF::SHT_LLVM_BB_ADDR_MAP_V0)
    OS << "llvm_bb_addr_map_v0";
  else if (Type == ELF::SHT_LLVM_OFFLOADING)
    OS << "llvm_offloading";
  else
    report_fatal_error("unsupported type 0x" + Twine::utohexstr(Type) +
                       " for section " + getName());

  if (EntrySize) {
    assert(Flags & ELF::SHF_MERGE);
    OS << "," << EntrySize;
  }

  if (Flags & ELF::SHF_GROUP) {
    OS << ",";
    printName(OS, Group.getPointer()->getName());
    if (isComdat())
      OS << ",comdat";
  }

  if (Flags & ELF::SHF_LINK_ORDER) {
    OS << ",";
    if (LinkedToSym)
      printName(OS, LinkedToSym->getName());
    else
      OS << '0';
  }

  if (isUnique())
    OS << ",unique," << UniqueID;

  OS << '\n';

  if (Subsection) {
    OS << "\t.subsection\t";
    Subsection->print(OS, &MAI);
    OS << '\n';
  }
}

bool MCSectionELF::useCodeAlign() const {
  return getFlags() & ELF::SHF_EXECINSTR;
}

bool MCSectionELF::isVirtualSection() const {
  return getType() == ELF::SHT_NOBITS;
}

StringRef MCSectionELF::getVirtualSectionKind() const { return "SHT_NOBITS"; }
