SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR RISCV)

set(TOOLCHAIN_PREFIX riscv64-unknown-elf-)
if(NOT TOOLCHAIN_PATH)
    message(FATAL_ERROR "TOOLCHAIN_PATH not defined")
endif()

set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES TOOLCHAIN_PATH)

message(STATUS "TOOLCHAIN_PATH:${TOOLCHAIN_PATH}")
# specify cross compilers and tools
SET(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
set(CMAKE_LINKER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}ld${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
set(CMAKE_OBJCOPY ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}objcopy CACHE INTERNAL "")
set(CMAKE_OBJDUMP ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}objdump CACHE INTERNAL "")
set(SIZE ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}size CACHE INTERNAL "")

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})
# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
