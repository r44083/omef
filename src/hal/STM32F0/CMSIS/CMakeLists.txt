set(TARGET cmsis)

if(NOT DEFINED DEVICE_DEF)
    message(FATAL_ERROR "DEVICE_DEF is not defined. Example: STM32F072xB")
endif()

add_library(${TARGET} STATIC
    # Select startup file (Device/STM32F0xx/Source/*.s) depending on DEVICE_DEF
    Device/STM32F0xx/Source/startup_$<LOWER_CASE:${DEVICE_DEF}>.s
    
    Device/STM32F0xx/Source/system_stm32f0xx.c
    syscalls.c
)

target_include_directories(${TARGET}
    PUBLIC
        Include
    PRIVATE
        Device/STM32F0xx/Include
)

target_compile_definitions(${TARGET}
    PUBLIC
        ${DEVICE_DEF}
    PRIVATE
        HSE_VALUE=8000000
)