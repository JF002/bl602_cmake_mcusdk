cmake_minimum_required(VERSION 3.19)

add_definitions(-Dbl602_iot)
add_definitions(-Dbl602)
set(CHIP bl602)
set(BOARD bl602_iot)
set(SUPPORT_FLOAT "n")
set(SUPPORT_SHELL "n")

set(CMAKE_TOOLCHAIN_FILE cmake/riscv64-unknown-elf-gcc.cmake)
include(sdk/bl_mcu_sdk/drivers/${CHIP}_driver/cpu_flags.cmake)
include(sdk/bl_mcu_sdk/tools/cmake/compiler_flags.cmake)
include(sdk/bl_mcu_sdk/tools/cmake/tools.cmake)

project(bl602_cmake_mcusdk C CXX ASM)
set(CMAKE_CXX_STANDARD 14)

add_executable(bl602_cmake_mcusdk
    sdk/bl_mcu_sdk/bsp/board/bl602/board.c
    sdk/bl_mcu_sdk/bsp/bsp_common/platform/bflb_platform.c
    sdk/bl_mcu_sdk/bsp/bsp_common/platform/syscalls.c

    drivers/Spi.cpp
    drivers/SpiMaster.cpp

    sx126x/SX126x.cpp

    main.cpp)

target_link_libraries(bl602_cmake_mcusdk common bl602_driver freertos)

include_directories(sdk/bl_mcu_sdk/bsp/bsp_common/platform)
include_directories(sdk/bl_mcu_sdk/bsp/board/bl602)
include_directories(bl602_cmake_mcusdk libs)

add_compile_options(${GLOBAL_C_FLAGS})
add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=c99>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++14>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-nostdlib>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>)
target_compile_options(bl602_cmake_mcusdk PUBLIC
    $<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:DEBUG>>: ${GLOBAL_C_FLAGS} -Og -g3 -std=c99>
    $<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:RELEASE>>: ${GLOBAL_C_FLAGS} -Os -std=c99>
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:DEBUG>>: ${GLOBAL_C_FLAGS} -Og -g3 -fno-rtti -nostdlib>
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:RELEASE>>: ${GLOBAL_C_FLAGS} -Os -fno-rtti -nostdlib>
    #$<$<COMPILE_LANGUAGE:ASM>: -MP -MD -x assembler-with-cpp>
    )

SET(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/sdk/bl_mcu_sdk/drivers/bl602_driver/bl602_flash.ld)
set(MAP_FILE ${CMAKE_CURRENT_BINARY_DIR}/bl602_cmake_mcusdk.map)
target_link_options(bl602_cmake_mcusdk PRIVATE ${GLOBAL_LD_FLAGS})
set_target_properties(bl602_cmake_mcusdk PROPERTIES LINK_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${MAP_FILE}")
set_target_properties(bl602_cmake_mcusdk PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

add_dependencies(bl602_cmake_mcusdk bl602_driver)
target_link_libraries(bl602_cmake_mcusdk bl602_driver c)
target_link_libraries(bl602_cmake_mcusdk m)

set(BIN_FILE bl602_cmake_mcusdk.bin)
set(ASM_FILE bl602_cmake_mcusdk.asm)
set(HEX_FILE bl602_cmake_mcusdk.hex)
add_custom_command(TARGET bl602_cmake_mcusdk POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:bl602_cmake_mcusdk> ${BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:bl602_cmake_mcusdk> ${HEX_FILE}
    COMMENT "Generate ${BIN_FILE}")

add_subdirectory(sdk/bl_mcu_sdk/common)
add_subdirectory(sdk/bl_mcu_sdk/drivers/bl602_driver)
add_subdirectory(sdk/bl_mcu_sdk/components/freertos)
