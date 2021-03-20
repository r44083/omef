# Main cmake file for project based on STM32F4
cmake_minimum_required(VERSION 3.13)

# NOTE: if config-x86.cmake is used, no toolchain file is required
set(CMAKE_TOOLCHAIN_FILE cmake/toolchain-arm-gcc-cortex.cmake)

get_filename_component(CURRENT_DIRNAME "${CMAKE_CURRENT_LIST_DIR}" NAME)
project(${CURRENT_DIRNAME} C CXX ASM)

# For vscode intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug)
endif()

include(cmake/config-stm32f4.cmake)
