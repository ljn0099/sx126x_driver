add_library(sx126x_hal_pico INTERFACE)

target_sources(sx126x_hal_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/sx126x_hal_pico.c
)

target_include_directories(sx126x_hal_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(sx126x_hal_pico INTERFACE
    hardware_spi
    hardware_gpio
    pico_stdlib
)

target_include_directories(sx126x_hal_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
