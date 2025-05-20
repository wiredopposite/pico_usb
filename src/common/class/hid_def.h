#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_DTYPE_HID           0x21
#define USB_DTYPE_HID_REPORT    0x22

#define USB_REQ_HID_GET_REPORT   0x01    /* Request to get the current HID report from the device.*/
#define USB_REQ_HID_GET_IDLE     0x02    /* Request to get the current device idle count.*/
#define USB_REQ_HID_GET_PROTOCOL 0x03    /* Request to get the current HID report protocol mode.*/
#define USB_REQ_HID_SET_REPORT   0x09    /* Request to set the current HID report to the device.*/
#define USB_REQ_HID_SET_IDLE     0x0A    /* Request to set the device's idle count.*/
#define USB_REQ_HID_SET_PROTOCOL 0x0B    /* Request to set the current HID report protocol mode.*/

#define USB_REQ_HID_REPORT_TYPE_INPUT     0x01
#define USB_REQ_HID_REPORT_TYPE_OUTPUT    0x02
#define USB_REQ_HID_REPORT_TYPE_FEATURE   0x03

typedef struct __attribute__((packed)) {
    uint8_t     bLength;            /* Size of the descriptor, in bytes. */
    uint8_t     bDescriptorType;    /* Type of the descriptor, set to \ref USB_DTYPE_HID */
    uint16_t    bcdHID;             /* BCD encoded version that the HID descriptor and device complies to. */
    uint8_t     bCountryCode;       /* Country code of the localized device, or zero if universal. */
    uint8_t     bNumDescriptors;    /* Total number of HID report descriptors for the interface. */
    uint8_t     bDescriptorType0;   /* 1'st HID report descriptor type, set to \ref USB_DTYPE_HID_REPORT */
    uint16_t    wDescriptorLength0; /* 1'sr HID report descriptor length in bytes. */
} usb_desc_hid_t;

typedef struct __attribute__((packed)) {
    uint8_t     bDescriptorType;   /* HID report descriptor type, set to USB_DTYPE_HID_REPORT */
    uint16_t    wDescriptorLength; /* HID report descriptor length in bytes. */
} usb_desc_hid_type_t;

#ifdef __cplusplus
}
#endif