set(TARGET hal)

# Pico-sdk
if(CMAKE_BUILD_TYPE EQUAL Debug)
    set(PICO_DEOPTIMIZED_DEBUG 1)
endif()
set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk)
include(${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk/external/pico_sdk_import.cmake)
pico_sdk_init()

add_library(${TARGET} STATIC
)

target_include_directories(${TARGET} PUBLIC
    .
)

target_link_libraries(${TARGET}
    PUBLIC
        pico_stdlib
)
