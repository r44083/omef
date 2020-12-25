set(TARGET hal)

add_library(${TARGET} STATIC
    adc/adc.cpp
    dac/dac.cpp
    dma/dma.cpp
    exti/exti.cpp
    gpio/gpio.cpp
    pwm/pwm.cpp
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

target_link_libraries(${TARGET} PRIVATE cmsis)
target_link_libraries(${TARGET} PUBLIC freertos)