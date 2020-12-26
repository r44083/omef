# Main cmake file for project based on STM32F0

set(CPU_FLAGS
    -mcpu=cortex-m0
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

add_subdirectory(src/drv)
add_subdirectory(src/hal/STM32F0)

set(DEVICE_DEF STM32F072xB)
add_subdirectory(src/hal/STM32F0/CMSIS)
#add_subdirectory(src/hal/STM32F0/CMSIS/DSP)

add_subdirectory(src/third_party/FreeRTOS)

# libnmea is a drv/gps/nmea dependency
set(NMEA_BUILD_SHARED_LIB OFF CACHE BOOL "")
set(NMEA_BUILD_EXAMPLES OFF CACHE BOOL "")
set(NMEA_UNIT_TESTS OFF CACHE BOOL "")
add_subdirectory(src/third_party/libnmea)

add_subdirectory(src/third_party/FatFs)
add_subdirectory(src/third_party/printf)
add_subdirectory(src/ul/fatfs_diskio)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/common/assert.c)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -Wl,--gc-sections
    -specs=nano.specs
    -specs=nosys.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -T ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/STM32F0/STM32F072RBTx_FLASH.ld
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    drv
    hal
    cmsis
    #cmsis-dsp-lib
    freertos
    nmea
    fatfs
    printf
    fatfs_diskio
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMENT "Post build steps for ${CMAKE_PROJECT_NAME}"
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf "${CMAKE_PROJECT_NAME}.bin"
)

# Add targets for flashing, erasing, resetting and debugging
set(JLINK_PARAMS -device STM32F072RB -if SWD)
include(cmake/debug-probes/jlink.cmake)
