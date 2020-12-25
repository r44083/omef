set(TARGET cmsis)

if(NOT DEFINED ST_DEVICE_DEF)
    message(FATAL_ERROR "ST_DEVICE_DEF is not defined. Example: STM32F407xx")
endif()

add_library(${TARGET} STATIC
    # Select startup file (Device/STM32F4xx/Source/*.s) depending on ST_DEVICE_DEF
    Device/STM32F4xx/Source/startup_$<LOWER_CASE:${ST_DEVICE_DEF}>.s
    
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
        ${ST_DEVICE_DEF}
    PRIVATE
        HSE_VALUE=8000000
)