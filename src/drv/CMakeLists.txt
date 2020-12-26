set(TARGET drv)

add_library(${TARGET} STATIC
    dataflash/dataflash.cpp
    dht/dht.cpp
    di/di.cpp
    ds18b20/ds18b20.cpp
    enc/enc.cpp
    gps/nmea.cpp
    hd44780/hd44780.cpp
    nrf24l01/nrf24l01.cpp
    onewire/onewire.cpp
    sd/sd.cpp
    sd/sd_spi.cpp
    singlewire/singlewire.cpp
)

target_link_libraries(${TARGET}
    PUBLIC
        hal
        freertos
    PRIVATE
        nmea
        printf
)
