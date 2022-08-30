#include "GameBoy.h"
#include "CommonArgs.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/Options.h"
#include "llvm/Option/ArgList.h"

using namespace clang::driver;
using namespace clang::driver::toolchains;
using namespace clang;
using namespace llvm::opt;

GameBoyToolChain::RISCWToolChain(const Driver &D, const llvm::Triple &Triple,
                               const ArgList &Args)
    : ToolChain(D, Triple, Args) {
  // ProgramPaths are found via 'PATH' environment variable.
}

bool GameBoyToolChain::isPICDefault() const { return true; }

bool GameBoyToolChain::isPIEDefault() const { return false; }

bool GameBoyToolChain::isPICDefaultForced() const { return true; }

bool GameBoyToolChain::SupportsProfiling() const { return false; }

bool GameBoyToolChain::hasBlocksRuntime() const { return false; }