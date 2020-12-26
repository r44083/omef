if(NOT DEFINED ESPTOOL_PARAMS)
    message(FATAL_ERROR "ESPTOOL_PARAMS is not defined. Example: --chip esp8266 --port COM6")
endif()

add_custom_target(flash
    COMMENT "Programming ${CMAKE_PROJECT_NAME}.bin"
    COMMAND esptool ${ESPTOOL_PARAMS} --baud 512000 --before default_reset --after hard_reset write_flash 0x00000 ${CMAKE_PROJECT_NAME}-0x00000.bin 0x10000 ${CMAKE_PROJECT_NAME}-0x10000.bin
)

add_custom_target(erase
    COMMENT "Erasing"
    COMMAND esptool ${ESPTOOL_PARAMS} --baud 512000 --before default_reset --after hard_reset erase_flash
)

add_custom_target(reset
    COMMENT "Resetting NOT IMPLEMENTED"
)

add_custom_target(debug DEPENDS flash
    COMMENT "Debugging NOT IMPLEMENTED"
)
