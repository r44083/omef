if(NOT DEFINED JLINK_PARAMS)
    message(FATAL_ERROR "JLINK_PARAMS is not defined. Example: -device STM32F407VG -if SWD")
endif()

add_custom_target(flash
    COMMENT "Programming ${CMAKE_PROJECT_NAME}.bin"
    COMMAND echo h>script.jlink
    COMMAND echo loadbin ${CMAKE_PROJECT_NAME}.bin, 0 >>script.jlink
    COMMAND echo r>>script.jlink
    COMMAND echo q>>script.jlink
    COMMAND JLink ${JLINK_PARAMS} -speed auto -CommanderScript script.jlink
)

add_custom_target(erase
    COMMENT "Erasing"
    COMMAND echo erase>script.jlink
    COMMAND echo q>>script.jlink
    COMMAND JLink ${JLINK_PARAMS} -speed auto -CommanderScript script.jlink
)

add_custom_target(reset
    COMMENT "Resetting"
    COMMAND echo r>script.jlink
    COMMAND echo q>>script.jlink
    COMMAND JLink ${JLINK_PARAMS} -speed auto -CommanderScript script.jlink
)

add_custom_target(debug DEPENDS flash
    COMMENT "Debugging ${CMAKE_PROJECT_NAME}.bin with JLinkGDBServerCL"
    COMMAND JLinkGDBServerCL ${JLINK_PARAMS} -speed auto -strict -vd -NoGui -singlerun
#    COMMAND JLinkGDBserverCL ${JLINK_PARAMS} -speed auto -strict -vd -nogui -singlerun -rtos GDBServer/RTOSPlugin_FreeRTOS
)
