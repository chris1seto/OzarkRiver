set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# CPU-specific flags
add_definitions(-DARM_MATH_CM4)
set(COMMON_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mabi=aapcs -u _printf_float")
set(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
set(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS}")

set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

# For bare-metal compilers, need to set this:
#   https://github.com/ObKo/stm32-cmake/issues/42
SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(PREFIX arm-none-eabi-)

# Specify the cross compiler
find_program(CMAKE_C_COMPILER ${PREFIX}gcc)
find_program(CMAKE_CXX_COMPILER ${PREFIX}g++)
find_program(CMAKE_ASM_COMPILER ${PREFIX}gcc)
find_program(CMAKE_RANLIB ${PREFIX}gcc-ranlib)
find_program(CMAKE_OBJCOPY ${PREFIX}objcopy)
find_program(CMAKE_OBJDUMP ${PREFIX}objdump)
find_program(CMAKE_AR ${PREFIX}gcc-ar)
find_program(CMAKE_ELF_SIZE ${PREFIX}size)

# Don't search for programs compiled for the ARM architecture
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM never)

# Only find libraries and include directories built for the ARM architecture
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY only)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE only)
