set(TARGET hal)

add_subdirectory(nRF5_SDK)

add_library(${TARGET} STATIC
    gpio/gpio.cpp
)

target_include_directories(${TARGET} PUBLIC
    .
)

target_link_libraries(${TARGET}
    PRIVATE
        nrf5_sdk
)
