#include "GameBoy.h"
#include "clang/Basic/MacroBuilder.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang;
using namespace clang::targets;

const char *const GameBoyTargetInfo::GCCRegNames[] = {
    // We only have integer registers
    "a", "b", "c", "d", "h", "l", "f", "sp", "pc",
};

const TargetInfo::GCCRegAlias GCCRegAliases[] = {
    {{"accumulator"}, "A"},
    {{"B"}, "B"},
    {{"C"}, "C"},
    {{"D"}, "D"},
    {{"H"}, "H"},
    {{"L"}, "L"},
    {{"flags"}, "F"},
    {{"pc"}, "pc"},
    {{"sp"}, "sp"}
};

ArrayRef<const char*> GameBoyTargetInfo::getGCCRegNames() const {
    return llvm::makeArrayRef(GCCRegNames);
}

ArrayRef<TargetInfo::GCCRegAlias> GameBoyTargetInfo::getGCCRegAliases() const {
    return llvm::makeArrayRef(GCCRegAliases);
}

void GameBoyTargetInfo::getTargetDefines(   const LangOptions &Opts,
                                            MacroBuilder &Builder) const {
        // Define the __GAMEBOY__ macro when building for this target
        Builder.defineMacro("__GAMEBOY__");
    }