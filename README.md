# BL602_cmake_mcusdk
Simple project based on the [MCU SDK from Bouffalo](https://github.com/bouffalolab/bl_mcu_sdk) for the BL602 RISC-V MCU.

## Howto

**NOTE** : This project assumes [this PR](https://github.com/bouffalolab/bl_mcu_sdk/pull/10) have been merged in bl_mcu_sdk. In the meantime, you can just remove the first `include` line of `sdk/bl_mcu_sdk/tools/cmake/compiler_flags.cmake` : 

```patch
Index: tools/cmake/compiler_flags.cmake
IDEA additional info:
Subsystem: com.intellij.openapi.diff.impl.patch.CharsetEP
<+>UTF-8
===================================================================
diff --git a/tools/cmake/compiler_flags.cmake b/tools/cmake/compiler_flags.cmake
--- a/tools/cmake/compiler_flags.cmake	(revision 3f8f00e4dfb4752e8b44b00eb4883f2742b25c24)
+++ b/tools/cmake/compiler_flags.cmake	(date 1626722492087)
@@ -1,5 +1,3 @@
-include(${CMAKE_SOURCE_DIR}/drivers/${CHIP}_driver/cpu_flags.cmake)
-
 list(APPEND GLOBAL_C_FLAGS -Os -g3)
 list(APPEND GLOBAL_C_FLAGS -fshort-enums -fno-common -fms-extensions -ffunction-sections -fdata-sections -fstrict-volatile-bitfields -ffast-math)
 list(APPEND GLOBAL_C_FLAGS -Wall -Wshift-negative-value -Wchar-subscripts -Wformat -Wuninitialized -Winit-self -Wignored-qualifiers -Wunused -Wundef)
```

 * Download and unzip the RISC-V toolchain. I use [this one](https://github.com/sifive/freedom-tools/releases/tag/v2020.08.0) but any RISC-V toolchain should work.

 * Clone the repo and initialize the submodule

```bash
git clone git@github.com:JF002/bl602_cmake_mcusdk.git
cd bl602_cmake_mcusdk
git submodule init --update
```

 * Build the project

```bash
mkdir build
cd build
cmake -DTOOLCHAIN_PATH=<path to the bin folder of the toolchain> ..
make
```

 * The binaries are available (elf, .bin, .hex and .map) in the current directory.

## Flash on the BL602 

*Note* : I tested this on a [Pinecone devboard from Pine64](https://pine64.com/product/pinecone-bl602-evaluation-board/).

There are many flash tool available (see [Loading code on this page](https://wiki.pine64.org/wiki/Nutcracker). For this example, I use [blflash](https://github.com/spacemeowx2/blflash).

```bash
 blflash-linux-amd64 flash ./bl602_cmake_mcusdk.bin --port /dev/ttyUSB0 
```

## Serial console

This project outputs "Hello world" every second on the serial port. Here's how to open the serial port using `screen`:

```bash
screen /dev/ttyUSB0 2000000
```

You should see the output of the firmware:
```bash
  ____               __  __      _       _       _     
 |  _ \             / _|/ _|    | |     | |     | |    
 | |_) | ___  _   _| |_| |_ __ _| | ___ | | __ _| |__  
 |  _ < / _ \| | | |  _|  _/ _` | |/ _ \| |/ _` | '_ \ 
 | |_) | (_) | |_| | | | || (_| | | (_) | | (_| | |_) |
 |____/ \___/ \__,_|_| |_| \__,_|_|\___/|_|\__,_|_.__/ 

Build:21:23:34,Jul 19 2021
Copyright (c) 2021 Bouffalolab team
dynamic memory init success,heap size = 125 Kbyte 
Hello world !
Hello world !
Hello world !
Hello world !
Hello world !
...
```