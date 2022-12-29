// ===--- GameBoy.h - Declare GameBoy target feature support === //
// This file declares GameBoyTargetInfo objects.

#ifndef LLVM_CLANG_LIB_BASIC_TARGETS_GAMEBOY_H
#define LLVM_CLANG_LIB_BASIC_TARGETS_GAMEBOY_H

#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Support/Compiler.h"

namespace clang {
namespace targets {

class LLVM_LIBRARY_VISIBILITY GameBoyTargetInfo : public TargetInfo {
    static const char *const GCCRegNames[];

public:
    GameBoyTargetInfo(const llvm::Triple &Triple, const TargetOptions &Options) : TargetInfo(Triple) {
        // Description string has to be kept in sync with backend string at
        // llvm/lib/Target/GameBoy/GameBoyTargetMachine.cpp
        resetDataLayout("e"
                        // little endian
                        "-p:16:8"
                        "-i8:8"
                        "-i16:16"
                        "-i32:16"
                        "-i64:16"
                        "-f32:8"
                        "-f64:8"
                        "-n8-a:8");
        // TODO: WIDTHS
        SuitableAlign = 16;
        WCharType = SignedInt;
        WIntType = UnsignedInt;
        IntPtrType = SignedInt;
        PtrDiffType = SignedInt;
        SizeType = UnsignedInt;
        IntWidth = 16;
        IntAlign = 16;
    }

    void getTargetDefines(const LangOptions &Opts, MacroBuilder &Builder) const override;

    ArrayRef<const char *> getGCCRegNames() const override;

    ArrayRef<TargetInfo::GCCRegAlias> getGCCRegAliases() const override;

    BuiltinVaListKind getBuiltinVaListKind() const override {
        return TargetInfo::VoidPtrBuiltinVaList;
    }

    ArrayRef<Builtin::Info> getTargetBuiltins() const override {
        return None;
    }

    bool validateAsmConstraint( const char *&Name,
                                TargetInfo::ConstraintInfo &info) const override { 
        return false; 
    }

    const char *getClobbers() const override {
        return "";
    }

    CallingConvCheckResult checkCallingConvention(CallingConv CC) const override {
        switch(CC) {
        case CC_C:
        case CC_SDCC_V0:
        case CC_SDCC_V1:
            return CCCR_OK;
        default:
            return CCCR_Warning;
        }
    }

    // TODO: Need to use default calling convention here, but investigate
    // using this part to use the RGBASM/SDCC calling conv as default.
    // CallingConv getDefaultCallingConv() const override {
    //     return CC_SDCC_V0;
    // }
};

} // namespace targets
} // namespace clang

#endif