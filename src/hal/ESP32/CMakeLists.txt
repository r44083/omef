set(TARGET hal)

add_library(${TARGET} STATIC
    gpio/gpio.cpp
)

target_include_directories(${TARGET} PUBLIC
    .
    esp-idf/components/freertos/include/freertos
)

# esp-idf
execute_process(COMMAND git apply ../esp-idf.patch
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/esp-idf
    ERROR_QUIET
)
include(esp-idf/tools/cmake/idf.cmake)
idf_build_process(esp32
    SDKCONFIG ${CMAKE_BINARY_DIR}/sdkconfig
    COMPONENTS esp32 freertos esptool_py
)
idf_build_executable(${CMAKE_PROJECT_NAME})

target_link_libraries(${TARGET}
    PUBLIC
        idf::esp32
        idf::freertos
)
