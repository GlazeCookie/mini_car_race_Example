set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_GCC arm-none-eabi-gcc HINTS /opt/homebrew/bin /usr/local/bin REQUIRED)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy HINTS /opt/homebrew/bin /usr/local/bin REQUIRED)
find_program(ARM_SIZE arm-none-eabi-size HINTS /opt/homebrew/bin /usr/local/bin REQUIRED)

set(CMAKE_C_COMPILER "${ARM_GCC}")
set(CMAKE_ASM_COMPILER "${ARM_GCC}")
set(CMAKE_OBJCOPY "${ARM_OBJCOPY}")
set(CMAKE_SIZE "${ARM_SIZE}")

set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m3 -mthumb")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-mcpu=cortex-m3 -mthumb --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections")
