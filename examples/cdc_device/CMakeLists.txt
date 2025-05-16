add_executable(cdc_device
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

target_sources(cdc_device
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

target_include_directories(cdc_device
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(cdc_device
    PRIVATE
        pico_stdlib 
        pico_usb
)

target_compile_definitions(cdc_device
    PUBLIC         
        PICO_DEFAULT_UART=1
        PICO_DEFAULT_UART_TX_PIN=4
        PICO_DEFAULT_UART_RX_PIN=5
)

target_compile_definitions(pico_usb 
    PRIVATE
        USBD_DEVICES_MAX=1
        USB_LOG_ENABLE=1
        USB_LOG_LEVEL=1
)

pico_add_extra_outputs(cdc_device)