# Main cmake file for project based on Raspberry Pi RP2040

set(CPU_FLAGS
    -mcpu=cortex-m0plus
)

add_compile_options(
    ${CPU_FLAGS}
    -mthumb
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

include_directories(./src)

#add_subdirectory(src/drv)
add_subdirectory(src/hal/RP2040)
#add_subdirectory(src/third_party/FreeRTOS)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp)# src/common/assert.c)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -Wl,--gc-sections
    -specs=nano.specs
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    hal
)

set(CMAKE_EXECUTABLE_SUFFIX .elf)
# Generate .uf2, .elf using pico-sdk
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
)

# Add targets for flashing, erasing, resetting and debugging
set(JLINK_PARAMS -device RP2040_M0_0 -if SWD)
include(cmake/debug-probes/jlink.cmake)
