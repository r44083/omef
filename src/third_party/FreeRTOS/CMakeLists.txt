set(TARGET freertos)

# Select port by CPU_M_OPTIONS variable. It is defined in cmake toolchain file
if (CPU_M_OPTIONS MATCHES cortex-m0)
    set(PORT_FOLDER portable/ARM_CM0)
elseif(CPU_M_OPTIONS MATCHES cortex-m3)
    set(PORT_FOLDER portable/ARM_CM3)
elseif(CPU_M_OPTIONS MATCHES cortex-m4)
    set(PORT_FOLDER portable/ARM_CM4)
endif()

add_library(${TARGET} STATIC
    croutine.c
    event_groups.c
    heap_4.c
    list.c
    my_hooks.c
    queue.c
    stream_buffer.c
    tasks.c
    timers.c
    ${PORT_FOLDER}/port.c
)

target_include_directories(${TARGET} PUBLIC
    include
    ${PORT_FOLDER}
)