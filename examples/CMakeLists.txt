cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
project(pico_usb_examples C CXX ASM)

pico_sdk_init()

set(PICO_USB_DIR ${CMAKE_CURRENT_LIST_DIR}/../)
add_subdirectory(${PICO_USB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/pico_usb)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/cdc_device)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/hub_device)