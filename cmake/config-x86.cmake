# Main cmake file for project based on x86

set(CPU_FLAGS
    -march=x86-64
)

add_compile_options(
    ${CPU_FLAGS}
    -ffunction-sections
    -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

include_directories(./src)

#add_subdirectory(src/drv)
add_subdirectory(src/hal/x86)
add_subdirectory(src/third_party/FreeRTOS)

# libnmea is a drv/gps/nmea dependency
#set(NMEA_BUILD_SHARED_LIB OFF CACHE BOOL "")
#set(NMEA_BUILD_EXAMPLES OFF CACHE BOOL "")
#set(NMEA_UNIT_TESTS OFF CACHE BOOL "")
#add_subdirectory(src/third_party/libnmea)

#add_subdirectory(src/third_party/FatFs)
#add_subdirectory(src/third_party/printf)
#add_subdirectory(src/ul/fatfs_diskio)
#add_subdirectory(src/ul/syslog)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/common/assert.c)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    #drv
    hal
    #fatfs_diskio
    #syslog
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMENT "Post build steps for ${CMAKE_PROJECT_NAME}"
    COMMAND size $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
)
