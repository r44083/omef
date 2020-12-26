set(TARGET printf)

add_library(${TARGET} STATIC
    printf.c
)

target_compile_definitions(${TARGET} PRIVATE
    PRINTF_INCLUDE_CONFIG_H
)
