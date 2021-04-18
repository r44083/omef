set(TARGET hal_esp8266)

add_library(${TARGET} STATIC
    adc/adc.cpp
    gpio/gpio.cpp
    spi/spi.cpp
    tim/tim.cpp
    uart/uart.cpp
)

target_include_directories(${TARGET} PUBLIC
    .
)


# ESP8266_RTOS_SDK
include(ESP8266_RTOS_SDK/tools/cmake/idf.cmake)
idf_build_process(esp8266
    SDKCONFIG ${CMAKE_BINARY_DIR}/sdkconfig
    COMPONENTS esp8266 freertos esptool_py pthread spi_flash
)
idf_build_executable(${CMAKE_PROJECT_NAME})

target_link_libraries(${TARGET}
    PUBLIC
        idf::esp8266
        idf::freertos
        idf::spi_flash
    PRIVATE
        idf::pthread
)
