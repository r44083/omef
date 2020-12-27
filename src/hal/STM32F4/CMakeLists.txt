set(TARGET hal)

add_subdirectory(CMSIS)
#add_subdirectory(CMSIS/DSP)

add_library(${TARGET} STATIC
    dac/dac.cpp
    dma/dma.cpp
    exti/exti.cpp
    gpio/gpio.cpp
    i2c/i2c.cpp
    pwm/pwm.cpp
    rcc/rcc.cpp
    rtc/rtc.cpp
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
        #cmsis-dsp-lib
)
