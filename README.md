# LLVM-DMG: An LLVM Backend for the Nintendo Game Boy

## Overview
LLVM-DMG is a fork of LLVM 15 which provides a backend for the Nintendo Game Boy family of devices. This project forms the basis of my BSc Computer Science dissertation at the University of Warwick, and is the first project of its kind to successfully compile programs for the Game Boy using an LLVM frontend (Clang). The project is not affiliated with LLVM.

Due to the nature of the assignment, I cannot accept any pull requests for the time being. Please feel free to open any issues however and I will use this
as an additional guide on which features need to be implemented as a matter of priority. Many aspects of a typical compiler toolchain will not be supported
at this early stage, and this is an ongoing project which will hopefully grow with and around the community.

### Compatibility
Assembly files produced by the backend are compatible with the RGBDS suite of tools. The calling convention is also broadly compatible with SDCC, although as of February 2023, stack usage for function arguments and return values are unsupported. This is a goal in the project roadmap.

## Usage instructions
### Building the LLVM suite
The `instructions` branch will typically contain the most progress; they are merged to `main` once they are tested and verified.
The repository can be built by cloning all the code here and following these steps:

1. Set up your build system. I have chosen to use Ninja in this example, and am also compiling in Debug mode. You will need to compile in Debug if you want to generate DAG diagrams using `llc`.

> Adjust the flags as you see fit. Compiling with the following configuration takes roughly 15 minutes on a 14" MacBook Pro, and shouldn't require more than 16GB of RAM on Linux/Mac (I have not been able to compile on WSL2 using 16GB of RAM).

```
$ cd llvm-project/build

# For a Release mode build
$ cmake -G "Ninja" -DLLVM_ENABLE_PROJECTS="clang" -DLLVM_TARGETS_TO_BUILD="" -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="GameBoy" -DCMAKE_BUILD_TYPE="Release" ../llvm

# For a Debug mode build. I choose to build AVR too, but this is not necessary.
$ cmake -G "Ninja" -DLLVM_ENABLE_PROJECTS="clang" -DLLVM_TARGETS_TO_BUILD="AVR" -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="GameBoy" -DCMAKE_BUILD_TYPE="Debug" -DLLVM_ENABLE_ASSERTIONS="On"  ../llvm

```

2. Build using your favourite build system. Using Ninja is as easy as
```
$ ninja
```

### Prerequisites for building ROM files
Once you have built the backend, you will need the following tools to run your ROM files:
- RGBDS installation
- Game Boy emulator of choice
- LLVM IR frontend of your choice (Clang provided)

In order to use the backend, you will need to use your frontend of choice to produce LLVM IR. The Clang build provided here uses 8-bit and 16-bit data types only for integers and you need to ensure your frontend does the same; 32-bit operations are likely to be unsupported across most of the backend.

The following commands can be used to build a ROM file.

```
# Produce LLVM IR using the provided Clang build.
$ llvm-project/build/bin/clang -O2 -emit-llvm -c file.c -o file.bc --target=gameboy

# Disassemble the LLVM IR for debugging purposes.
$ llvm-project/build/bin/llvm-dis file.bc

# Produce the assembly file
$ llvm-project/build/bin/llc -march="Game Boy" -O2 filetype=asm file.bc -o file.asm

# Produce executable using RGBDS
$ rgbasm file.asm -o file.o
$ rgblink -o file.gb file.o
$ rgbfix -v -p 0xFF file.gb
```

## Acknowledgements
Many thanks to the gbdev and LLVM communities on Discord for their invaluable support throughout this project.
