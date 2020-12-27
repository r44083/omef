# Main cmake file for project based on nRF52

set(CPU_FLAGS
    -mcpu=cortex-m4
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
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
set(DEVICE_DEF NRF52832_XXAA)
add_subdirectory(src/hal/nRF)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/common/assert.c)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -Wl,--gc-sections
    -specs=nano.specs
    -specs=nosys.specs
    -nostartfiles
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -L ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/nRF/nRF5_SDK/modules/nrfx/mdk
    -T ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/nRF/nRF5_SDK/modules/nrfx/mdk/nrf52832_xxaa.ld
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    #drv
    hal
    nrf5_sdk
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMENT "Post build steps for ${CMAKE_PROJECT_NAME}"
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf "${CMAKE_PROJECT_NAME}.bin"
)

# Add targets for flashing, erasing, resetting and debugging
set(JLINK_PARAMS -device nRF52832_xxAA -if SWD)
include(cmake/debug-probes/jlink.cmake)
