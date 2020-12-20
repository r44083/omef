add_custom_target(flash
    COMMENT "Programming ${CMAKE_PROJECT_NAME}.bin"
    COMMAND STM32_Programmer_CLI --connect port=SWD --write ${CMAKE_PROJECT_NAME}.bin 0x08000000 --verify -rst
)

add_custom_target(erase
    COMMENT "Erasing"
    COMMAND STM32_Programmer_CLI --connect port=SWD --erase all
)

add_custom_target(reset
    COMMENT "Resetting"
    COMMAND STM32_Programmer_CLI --connect port=SWD -rst
)

# DEPENDS flash
add_custom_target(debug
    COMMENT "Debugging ${CMAKE_PROJECT_NAME}.bin with ST-LINK_gdbserver"
    COMMAND ST-LINK_gdbserver --swd --verbose --persistent --verify --port-number 2331 --stm32cubeprogrammer-path C:/mydevtools/stm32-tools/STM32CubeProgrammer/bin
)