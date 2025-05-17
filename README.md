## pico_usb
A flexible USB device stack for the Raspberry Pi Pico that supports both hardware USB and PIO-based USB implementations.

### Features
- Dual USB implementation support:
    - Use the Pico's built-in USB hardware controller
    - Use the PIO-based USB implementation
- PIO USB capabilities:
    - Emulate a USB hub with multiple downstream devices
    - Configurable number of devices
- Flexible configuration:
    - Per-device USB driver implementation, though generic class drivers can easily be implemented by the user
    - Adjustable endpoint count (USBD_ENDPOINTS_MAX)
    - Configurable maximum endpoint/buffer size for PIO (USBD_ENDPOINT_MAX_SIZE)
    - Customizable enumeration buffer size (USBD_ENUMERATION_SIZE)

### Usage
See ```examples/cdc_device``` for a basic CDC device example. ```examples/hub_device``` contains an example USB hub implementation via PIO with 2 downstream devices, one HID gamepad, and one CDC device.

- PIO USB Requirements
    - 27 OHM series resistors on DP and DP pins
    - 1.5K OHM pullup to +3V3 on the DP pin 
    - System clock must be set to 240MHz
    - If emulating a hub, first device initialized must be the hub
    - GPIO and PIO config can be customized (default pins are DP=GPIO0 DM=GPIO1)

### Device Configuration
These can be defined with CMake or before including the ```usbd.h``` header.
```c
// Maximum number of USB devices
#define USBD_DEVICES_MAX        1

// Maximum number of endpoints per device (max is 16)
#define USBD_ENDPOINTS_MAX      16

// Maximum endpoint size, used by PIO
#define USBD_ENDPOINT_MAX_SIZE  64

// Enumeration buffer size, for descriptors and control requests/responses
#define USBD_ENUMERATION_SIZE   256
```

### PIO Configuration
These are not externally defined in a header, but can be redefined via CMake
```c
// Data+ pin number
#define PIO_USBD_DP_PIN             0   

// Pinout order, PIO_USB_PINOUT_DPDM or PIO_USB_PINOUT_DMDP
#define PIO_USBD_PINOUT             PIO_USB_PINOUT_DPDM 

// TX PIO index
#define PIO_USBD_PIO_TX             0   

// RX PIO index
#define PIO_USBD_PIO_RX             1   

// TX state machine index
#define PIO_USBD_SM_TX              0   

// RX state machine index
#define PIO_USBD_SM_RX              0   

// RX End of Packet state machine index
#define PIO_USBD_SM_RX_EOP          1   
    
// TX DMA channel
#define PIO_USBD_DMA_CHAN_TX        0   
```

### Use with your CMake project
```cmake
cmake_minimum_required(VERSION 3.13)

# PICO_SDK_PATH from cmake environment, or set it here
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(example_project C CXX ASM)

pico_sdk_init()

# Add the pico_usb lib
add_subdirectory(path/to/pico_usb)

add_executable(example_project
    my_executables.c
)

target_link_libraries(example_project 
    PRIVATE
        pico_stdlib
        pico_usb
)

# Optional: Change your default UART pins if they conflict with the PIO USB pins
target_compile_definitions(example_project 
    PUBLIC
        PICO_DEFAULT_UART=1
        PICO_DEFAULT_UART_TX_PIN=4
        PICO_DEFAULT_UART_RX_PIN=5
)

# Optional: Define the things you want to change about pico_usb
target_compile_definitions(pico_usb 
    PUBLIC
        USBD_DEVICES_MAX=3
        PIO_USBD_DP_PIN=0
        PIO_USBD_PINOUT=PIO_USB_PINOUT_DPDM 
)

pico_add_extra_outputs(example_project)
```