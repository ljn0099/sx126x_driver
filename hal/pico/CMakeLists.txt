# hal/pico/CMakeLists.txt

# Este archivo asume que ya se incluyó pico_sdk_import.cmake desde tu aplicación principal
# o desde un nivel superior

add_library(sx126x_hal_pico INTERFACE)

target_sources(sx126x_hal_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/sx126x_hal_pico.c
)

target_include_directories(sx126x_hal_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Dependencias del Pico SDK
target_link_libraries(sx126x_hal_pico INTERFACE
    hardware_spi
    hardware_gpio
    pico_stdlib
)

# Opcional: Si quieres que cualquier target que use esta librería tenga acceso al SDK
target_include_directories(sx126x_hal_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
