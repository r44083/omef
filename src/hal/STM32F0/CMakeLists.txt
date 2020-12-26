set(TARGET hal)

add_library(${TARGET} STATIC
    dma/dma.cpp
    exti/exti.cpp
    gpio/gpio.cpp
    rcc/rcc.cpp
    spi/spi.cpp
    systick/systick.cpp
    tim/tim.cpp
    uart/uart.cpp
    wdt/wdt.cpp
)

target_include_directories(${TARGET} PUBLIC
    .
)

target_link_libraries(${TARGET}
    PUBLIC
        freertos
    PRIVATE
        cmsis
)
