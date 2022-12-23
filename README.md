# LLVM-DMG: An LLVM Backend for the Nintendo Game Boy

This repository is a fork of the LLVM repository, and has not been kept
up to date. The fork was made roughly just before the release of LLVM 16.
This is an interim solution- eventually, at a convenient point in time,
a new repository will be created to hold the relevant files for the Game Boy
backend to avoid any fork related issues.

## Overview
This repository contains code for the Game Boy backend for LLVM. It currently only
supports Clang, and does not have official documentation at the moment. I am
writing this project as my dissertation for my BSc in Computer Science, and hence
cannot accept any PRs or code contributions until the assessment is complete
(roughly August 2023).

The backend currently targets RGBASM. See below for a list of supported and unsupported features.

## Usage instructions
### Building the LLVM suite
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

## Features
### What this project is _not_
This is not a finished project. My dissertation has specific requirements which means that there are various optimisations and QOL features that I would prefer to be here that aren't. There are also likely to be a few unsupported features for the Game Boy which should be implemented by any mature compiler toolchain; the goal is to complete as many of these as possible, but the priority is to provide a broad starting point for a continuously developed project.