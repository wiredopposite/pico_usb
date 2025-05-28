cmake_minimum_required(VERSION 3.13)
project(pico_usb C CXX ASM)

set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/Pico-PIO-USB)

add_library(pico_usb STATIC
    ${SRC_DIR}/usbd/usbd.c
    ${SRC_DIR}/usbd/dcd/dcd_pico.c
    ${SRC_DIR}/usbd/dcd/dcd_pio.cpp
)

target_include_directories(pico_usb PUBLIC
    ${SRC_DIR}
)

target_link_libraries(pico_usb PUBLIC
    pico_stdlib
    pico_sync
    pico_unique_id
	hardware_structs
	hardware_irq
	hardware_resets
	pico_platform
    pico_pio_usb
)

target_compile_options(pico_usb PRIVATE
    -O3
)