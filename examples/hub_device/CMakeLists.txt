add_executable(hub_device
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
    ${CMAKE_CURRENT_LIST_DIR}/src/hid.c
    ${CMAKE_CURRENT_LIST_DIR}/src/cdc.c
)

target_sources(hub_device
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/main.c
        ${CMAKE_CURRENT_LIST_DIR}/src/hid.c
        ${CMAKE_CURRENT_LIST_DIR}/src/cdc.c
)

target_include_directories(hub_device
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(hub_device
    PRIVATE
        pico_stdlib 
        pico_usb
)

target_compile_definitions(hub_device
    PUBLIC
        # PIO will be using GPIO 0 and 1 for USB
        PICO_DEFAULT_UART=1
        PICO_DEFAULT_UART_TX_PIN=4
        PICO_DEFAULT_UART_RX_PIN=5
)

target_compile_definitions(pico_usb
    PUBLIC
        USBD_DEVICES_MAX=3
)

pico_add_extra_outputs(hub_device)