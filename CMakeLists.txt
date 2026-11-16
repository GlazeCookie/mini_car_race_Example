cmake_minimum_required(VERSION 3.20)
project(micro_smartcar LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

file(GLOB APP_SOURCES CONFIGURE_DEPENDS "Core/code/*.c")
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS "Drivers/STM32F1xx_HAL_Driver/Src/*.c")

add_executable(micro_smartcar
    Core/Src/main.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/stm32f1xx_it.c
    Core/Src/system_stm32f1xx.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Startup/startup_stm32f103c8tx.s
    cmake/uart_stdio.c
    ${APP_SOURCES}
    ${HAL_SOURCES}
)

target_compile_definitions(micro_smartcar PRIVATE USE_HAL_DRIVER STM32F103xB)
target_include_directories(micro_smartcar PRIVATE
    Core/Inc
    Core/code
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
)
target_compile_options(micro_smartcar PRIVATE $<$<CONFIG:Debug>:-Og>)
target_link_options(micro_smartcar PRIVATE
    "-T${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld"
    "-Wl,-Map=${CMAKE_BINARY_DIR}/micro_smartcar.map"
    -Wl,--print-memory-usage
)
set_property(TARGET micro_smartcar APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld")
target_link_libraries(micro_smartcar PRIVATE m)

add_custom_command(TARGET micro_smartcar POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:micro_smartcar>" "${CMAKE_BINARY_DIR}/micro_smartcar.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:micro_smartcar>" "${CMAKE_BINARY_DIR}/micro_smartcar.bin"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:micro_smartcar>"
    VERBATIM
)
