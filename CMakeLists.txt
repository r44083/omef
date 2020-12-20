cmake_minimum_required(VERSION 3.5)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm-gcc-cortex.cmake)

get_filename_component(CURRENT_DIRNAME "${CMAKE_CURRENT_LIST_DIR}" NAME)
project(${CURRENT_DIRNAME} C CXX ASM)

# For vscode intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug)
endif()

include_directories(./src)

add_subdirectory(src/drv)
add_subdirectory(src/hal/STM32F4)
add_subdirectory(src/hal/STM32F4/CMSIS)
#add_subdirectory(src/hal/STM32F4/CMSIS/DSP)
add_subdirectory(src/third_party/FreeRTOS)

# libnmea is a drv/gps/nmea dependency
set(NMEA_BUILD_SHARED_LIB OFF CACHE BOOL "")
set(NMEA_BUILD_EXAMPLES OFF CACHE BOOL "")
set(NMEA_UNIT_TESTS OFF CACHE BOOL "")
add_subdirectory(src/third_party/libnmea)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/common/assert.c)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    drv
    hal
    cmsis
    #cmsis-dsp-lib
    freertos
    nmea
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMENT "Post build steps for ${CMAKE_PROJECT_NAME}"
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf "${CMAKE_PROJECT_NAME}.bin"
)

# Add targets for flashing, erasing, resetting and debugging
set(JLINK_PARAMS -device STM32F407VG -if SWD)
include(cmake/debug-probes/jlink.cmake)