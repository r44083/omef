set(TARGET hal)

add_library(${TARGET} STATIC
    gpio/gpio.cpp
)

target_include_directories(${TARGET} PUBLIC
    .
)

target_link_libraries(${TARGET}
    PUBLIC
        freertos
)
