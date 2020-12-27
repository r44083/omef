set(TARGET freertos)

# Select FreeRTOS port by COMPILE_OPTIONS
get_property(COMPILE_OPTIONS DIRECTORY . PROPERTY COMPILE_OPTIONS)
if (COMPILE_OPTIONS MATCHES cortex-m0)
    set(PORT_FOLDER portable/ARM_CM0)
elseif(COMPILE_OPTIONS MATCHES cortex-m3)
    set(PORT_FOLDER portable/ARM_CM3)
elseif(COMPILE_OPTIONS MATCHES cortex-m4)
    set(PORT_FOLDER portable/ARM_CM4)
elseif(COMPILE_OPTIONS MATCHES x86-64)
    set(PORT_FOLDER portable/MSVC-MingW)
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

if(COMPILE_OPTIONS MATCHES x86-64)
    # Windows library which implements timers
    target_link_libraries(${TARGET} winmm)
endif()
