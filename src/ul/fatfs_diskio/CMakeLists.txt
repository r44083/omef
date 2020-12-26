set(TARGET fatfs_diskio)

add_library(${TARGET} STATIC
    fatfs_diskio.cpp
    fatfs_diskio_dataflash.cpp
    fatfs_diskio_sd.cpp
)

target_link_libraries(${TARGET}
    PUBLIC
        drv
        fatfs
)
