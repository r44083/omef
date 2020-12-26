if(NOT DEFINED OPENOCD_PARAMS)
    message(FATAL_ERROR "OPENOCD_PARAMS is not defined. Example: -f interface/stlink-v2.cfg transport select hla_swd -f target/stm32f4x.cfg")
endif()

add_custom_target(flash
    COMMENT "Programming ${CMAKE_PROJECT_NAME}.bin"
    COMMAND openocd ${OPENOCD_PARAMS} -c "init; halt; program ${CMAKE_PROJECT_NAME}.bin 0x08000000 verify; reset run; exit"
)

add_custom_target(erase
    COMMENT "Erasing"
    COMMAND openocd ${OPENOCD_PARAMS} -c "init; halt; stm32f4x mass_erase 0; exit"
)

add_custom_target(reset
    COMMENT "Resetting"
    COMMAND openocd ${OPENOCD_PARAMS} -c "init; reset run; exit"
)

add_custom_target(debug DEPENDS flash
    COMMENT "Debugging ${CMAKE_PROJECT_NAME}.bin with openocd"
    COMMAND openocd ${OPENOCD_PARAMS} -c "gdb_port 2331; debug_level 2; set WORKAREASIZE 0x2000; reset_config srst_only"
)
