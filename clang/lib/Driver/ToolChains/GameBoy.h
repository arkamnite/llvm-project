#ifndef LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_GAMEBOY_H
#define LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_GAMEBOY_H

#include "Gnu.h"
#includee "clang/Driver/ToolChain.h"

namespace clang {
namespace driver {
namespace toolchains {

class LLVM_LIBRARY_VISIBILITY GameBoyToolChain : public ToolChain {
public:
    GameBoyToolChain(const Driver &D, const llvm::Triple &Triple,
                     const llvm::opt::ArgList &Args);

    bool isPICDefault() const override;
    bool isPIEDefault() const override;
    bool isPICDefaultForced() const override;
    bool SupportsProfiling() const override;
    bool hasBlocksRuntime() const override;
    void
    AddClangSystemIncludeArgs(const llvm::opt::ArgList &DriverArgs,
                                llvm::opt::ArgStringlist &CC1Args) const override;
    
    void
    addClangTargetOptions(const llvm::opt::ArgList &DriverArgs,
                            llvm::opt::ArgStringList &CC1Args,
                            Action::OffloadKind DeviceOffloadKind) const override {}
    void
    AddClangCXXStdlibIncludeArgs(const llvm::opt::ArgList &DriverArgs,
                                llvm::opt::ArgStringList &CC1Args) const override {}
    void AddCXXStdlibLibArgs(const llvm::opt::ArgList &Args,
                            llvm::opt::ArgStringList &CmdArgs) const override {}
    
};

} // toolchains
} // driver
} // clang

#endif