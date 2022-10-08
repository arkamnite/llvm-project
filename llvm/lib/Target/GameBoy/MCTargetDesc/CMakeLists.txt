add_llvm_component_library(LLVMGameBoyDesc
  GameBoyAsmBackend.cpp
  GameBoyELFObjectWriter.cpp
  GameBoyELFStreamer.cpp
  GameBoyInstPrinter.cpp
  GameBoyMCAsmInfo.cpp
  GameBoyMCCodeEmitter.cpp
  GameBoyMCELFStreamer.cpp
  GameBoyMCExpr.cpp
  GameBoyMCTargetDesc.cpp
  GameBoyTargetStreamer.cpp

  LINK_COMPONENTS
  MC
  GameBoyInfo
  Support

  ADD_TO_COMPONENT
  GameBoy
)

add_dependencies(LLVMGameBoyDesc GameBoyCommonTableGen)

