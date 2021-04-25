# Main cmake file for project based on ESP32

add_compile_options(
    $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:-mlongcalls>
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

include_directories(./src)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/common/assert.c)

#add_subdirectory(src/drv)
add_subdirectory(src/hal/ESP32)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -nostdlib
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    hal
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}
)

# Add targets for flashing, erasing, resetting and debugging
set(ESPTOOL_PARAMS --chip esp32 --port COM3)
include(cmake/debug-probes/esptool.cmake)
