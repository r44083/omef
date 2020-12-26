set(TARGET syslog)

add_library(${TARGET} STATIC
    syslog.cpp
)

target_link_libraries(${TARGET}
    PUBLIC
        freertos
    PRIVATE
        printf
)
