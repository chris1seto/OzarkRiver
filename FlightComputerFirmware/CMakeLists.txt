cmake_minimum_required(VERSION 3.2)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/ArmGccCortexM4HF.cmake")

project(OzarkRiver ASM C CXX)

SET(LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/Platform/STM32_40K_256K_FLASH.ld")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections --specs=nosys.specs -T ${LINKER_SCRIPT}")

add_definitions(-DUSE_FREERTOS -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER)
add_compile_options(
    -DSTM32F405xx
    -fno-builtin
    -Wall
    -ffunction-sections
    -fdata-sections
    -fomit-frame-pointer
    -fno-unroll-loops
    -ffast-math
    -ftree-vectorize
    -flto
)

set(CMAKE_C_STANDARD 99)

add_subdirectory(HalF4)
add_subdirectory(FreeRTOS)
add_subdirectory(Cmsis)
add_subdirectory(Platform)

# Get project sources
file(GLOB_RECURSE PROJECT_SOURCES "Src/*.c")

add_executable(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES})

# Project includes
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC Src)

target_link_libraries(${CMAKE_PROJECT_NAME} HalF4)
target_link_libraries(${CMAKE_PROJECT_NAME} FreeRTOS)
target_link_libraries(${CMAKE_PROJECT_NAME} Cmsis)
target_link_libraries(${CMAKE_PROJECT_NAME} Platform)

# Generate ihex for a target
function(generate_ihex target_name)
  add_custom_command(TARGET ${target_name}
    POST_BUILD
    COMMAND
      ${CMAKE_OBJCOPY}
    ARGS
      "${target_name}.elf"
      "-O"
      "ihex"
      "${target_name}.hex"
    COMMENT
      "Generating ihex for target ${target_name}"
  )
endfunction()

# Generate bin for a target
function(generate_bin target_name)
  add_custom_command(TARGET ${target_name}
    POST_BUILD
    COMMAND
      ${CMAKE_OBJCOPY}
    ARGS
      "${target_name}.elf"
      "--gap-fill"
      "0xff"
      "-O"
      "binary"
      "${target_name}.bin"
    COMMENT
      "Generating bin for target ${target_name}"
  )
endfunction()

generate_bin(${CMAKE_PROJECT_NAME})
generate_ihex(${CMAKE_PROJECT_NAME})