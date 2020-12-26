set(TARGET cmsis)

if(NOT DEFINED DEVICE_DEF)
    message(FATAL_ERROR "DEVICE_DEF is not defined. Example: STM32F407xx")
endif()

add_library(${TARGET} STATIC
    Device/STM32F4xx/Source/startup_$<LOWER_CASE:${DEVICE_DEF}>.s
    Device/STM32F4xx/Source/system_stm32f4xx.c
)

target_include_directories(${TARGET}
    PUBLIC
        Include
    PRIVATE
        Device/STM32F4xx/Include
)

target_compile_definitions(${TARGET}
    PUBLIC
        ${DEVICE_DEF}
    PRIVATE
        HSE_VALUE=8000000
)
