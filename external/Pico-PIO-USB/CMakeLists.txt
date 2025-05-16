cmake_minimum_required(VERSION 3.13)
project(pico_pio_usb C CXX ASM)

set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

add_library(pico_pio_usb STATIC
    ${SRC_DIR}/usb_crc.c
)

pico_generate_pio_header(pico_pio_usb ${SRC_DIR}/usb_tx.pio)
pico_generate_pio_header(pico_pio_usb ${SRC_DIR}/usb_rx.pio)

target_include_directories(pico_pio_usb PUBLIC
    ${SRC_DIR}
)

target_link_libraries(pico_pio_usb PUBLIC
    pico_stdlib
    pico_sync
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_resets
)

target_compile_options(pico_pio_usb PRIVATE
    -O3
)