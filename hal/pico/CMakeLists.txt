add_library(sx126x_hal STATIC
    sx126x_hal_pico.c
)

target_include_directories(sx126x_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(sx126x_hal PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../src
)

target_link_libraries(sx126x_hal PUBLIC
    hardware_spi
    hardware_gpio
    pico_stdlib
)
