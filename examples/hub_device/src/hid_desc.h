#pragma once

#include <stdint.h>
#include "usb_def.h"
#include "usb_util.h"
#include "class/hid_def.h"  

#define HID_CTRL_EP_SIZE    64U

#define HID_EPADDR_OUT      (2U | USB_EP_DIR_OUT)
#define HID_EPADDR_IN       (1U | USB_EP_DIR_IN)
#define HID_EP_SIZE_IN      64U
#define HID_EP_SIZE_OUT     64U

#define HID_JOYSTICK_MID     0x80

#define HID_DPAD_UP          0x00
#define HID_DPAD_UP_RIGHT    0x01
#define HID_DPAD_RIGHT       0x02
#define HID_DPAD_DOWN_RIGHT  0x03
#define HID_DPAD_DOWN        0x04
#define HID_DPAD_DOWN_LEFT   0x05
#define HID_DPAD_LEFT        0x06
#define HID_DPAD_UP_LEFT     0x07
#define HID_DPAD_CENTER      0x08

#define HID_BUTTONS_Y        (1U <<  0)
#define HID_BUTTONS_B        (1U <<  1)
#define HID_BUTTONS_A        (1U <<  2)
#define HID_BUTTONS_X        (1U <<  3)
#define HID_BUTTONS_L        (1U <<  4)
#define HID_BUTTONS_R        (1U <<  5)
#define HID_BUTTONS_ZL       (1U <<  6)
#define HID_BUTTONS_ZR       (1U <<  7)
#define HID_BUTTONS_MINUS    (1U <<  8)
#define HID_BUTTONS_PLUS     (1U <<  9)
#define HID_BUTTONS_L3       (1U << 10)
#define HID_BUTTONS_R3       (1U << 11)
#define HID_BUTTONS_HOME     (1U << 12)
#define HID_BUTTONS_CAPTURE  (1U << 13)

static const usb_desc_device_t HID_DESC_DEVICE = {
    .bLength            = sizeof(usb_desc_device_t),
    .bDescriptorType    = USB_DTYPE_DEVICE,
    .bcdUSB             = USB_BCD_VERSION_2_0,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = HID_CTRL_EP_SIZE,
    .idVendor           = 0x0F0D,
    .idProduct          = 0x0092,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

static const uint8_t HID_DESC_REPORT[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x35, 0x00,        //   Physical Minimum (0)
    0x45, 0x01,        //   Physical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x10,        //   Report Count (16)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x10,        //   Usage Maximum (0x10)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x25, 0x07,        //   Logical Maximum (7)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
    0x09, 0x39,        //   Usage (Hat switch)
    0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x65, 0x00,        //   Unit (None)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x46, 0xFF, 0x00,  //   Physical Maximum (255)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x20,        //   Usage (0x20)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x0A, 0x21, 0x26,  //   Usage (0x2621)
    0x95, 0x08,        //   Report Count (8)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection
};

typedef struct __attribute__((packed)) {
    usb_desc_config_t   config;
    usb_desc_itf_t      hid_itf;
    usb_desc_hid_t      hid_desc;
    usb_desc_endpoint_t hid_ep_out;
    usb_desc_endpoint_t hid_ep_in;
} usb_hid_desc_config_t;

static const usb_hid_desc_config_t HID_DESC_CONFIG = {
    .config = {
        .bLength                = sizeof(usb_desc_config_t),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(usb_hid_desc_config_t),
        .bNumInterfaces         = 1,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = USB_ATTR_RESERVED | USB_ATTR_SELF_POWERED,
        .bMaxPower              = 50, // 100mA
    },
    .hid_itf = {
        .bLength                = sizeof(usb_desc_itf_t),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_HID,
        .bInterfaceSubClass     = 0x00,
        .bInterfaceProtocol     = 0x00,
        .iInterface             = 0
    },
    .hid_desc = {
        .bLength                = sizeof(usb_desc_hid_t),
        .bDescriptorType        = USB_DTYPE_HID,
        .bcdHID                 = 0x0111,
        .bCountryCode           = 0x00,
        .bNumDescriptors        = 1,
        .bDescriptorType0       = USB_DTYPE_HID_REPORT,
        .wDescriptorLength0     = sizeof(HID_DESC_REPORT),
    },
    .hid_ep_out = {
        .bLength                = sizeof(usb_desc_endpoint_t),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = HID_EPADDR_OUT,
        .bmAttributes           = USB_EP_TYPE_INTERRUPT,
        .wMaxPacketSize         = HID_EP_SIZE_OUT,
        .bInterval              = 1,
    },
    .hid_ep_in = {
        .bLength                = sizeof(usb_desc_endpoint_t),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = HID_EPADDR_IN,
        .bmAttributes           = USB_EP_TYPE_INTERRUPT,
        .wMaxPacketSize         = HID_EP_SIZE_IN,
        .bInterval              = 1,
    },
};

static const usb_desc_string_t HID_DESC_STR_0 = USB_ARRAY_DESC(0x0409);
static const usb_desc_string_t HID_DESC_STR_1 = USB_STRING_DESC("HORI CO.,LTD.");
static const usb_desc_string_t HID_DESC_STR_2 = USB_STRING_DESC("POKKEN CONTROLLER");
static const usb_desc_string_t* HID_DESC_STR[] = {
    &HID_DESC_STR_0,
    &HID_DESC_STR_1,
    &HID_DESC_STR_2,
};

typedef struct __attribute__((packed, aligned(4))) {
    uint16_t buttons;
    uint8_t dpad;
    uint8_t joystick_lx;
    uint8_t joystick_ly;
    uint8_t joystick_rx;
    uint8_t joystick_ry;
    uint8_t vendor;
} hid_report_in_t;