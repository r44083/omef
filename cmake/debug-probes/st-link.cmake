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

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL Windows)
    execute_process(COMMAND cmd /c "where STM32_Programmer_CLI" OUTPUT_VARIABLE STM32_Programmer_CLI_PATH)
else()
    execute_process(COMMAND sh "which STM32_Programmer_CLI" OUTPUT_VARIABLE STM32_Programmer_CLI_PATH)
endif()
get_filename_component(STM32_Programmer_CLI_DIRNAME "${STM32_Programmer_CLI_PATH}" DIRECTORY)

add_custom_target(debug DEPENDS flash
    COMMENT "Debugging ${CMAKE_PROJECT_NAME}.bin with ST-LINK_gdbserver"
    COMMAND ST-LINK_gdbserver --swd --verbose --persistent --verify --port-number 2331
        --stm32cubeprogrammer-path ${STM32_Programmer_CLI_DIRNAME}
)
