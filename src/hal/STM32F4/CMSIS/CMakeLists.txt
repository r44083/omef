set(TARGET cmsis)

add_library(${TARGET} STATIC
    Device/STM32F4xx/Source/startup_stm32f407xx.s
    Device/STM32F4xx/Source/system_stm32f4xx.c
    syscalls.c
)

target_include_directories(${TARGET}
    PUBLIC
        Include
    PRIVATE
        Device/STM32F4xx/Include
)

target_compile_definitions(${TARGET}
    PUBLIC
        STM32F407xx
    PRIVATE
        HSE_VALUE=8000000
)