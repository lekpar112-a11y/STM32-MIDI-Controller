cmake_minimum_required(VERSION 3.10)
project(STM32_MIDI_Controller C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# MCU
set(MCU cortex-m3)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C6_FLASH.ld)

# Source
file(GLOB SRC src/*.cpp src/*.c)

add_executable(${PROJECT_NAME}.elf ${SRC})

target_include_directories(${PROJECT_NAME}.elf PRIVATE include)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${LINKER_SCRIPT}
    -mcpu=${MCU}
    -mthumb
    -Wl,--gc-sections
)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
    -mcpu=${MCU}
    -mthumb
    -O2
)
