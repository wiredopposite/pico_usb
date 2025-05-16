#pragma once

#include <stdint.h>
#include "usb_util.h"
#include "usb_def.h"
#include "class/cdc_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CDC_CTRL_EP_SIZE        64U

#define CDC_NOTIF_EPADDR_IN     (3U | USB_EP_DIR_IN)
#define CDC_NOTIF_EP_SIZE_IN    8U

#define CDC_DATA_EPADDR_OUT     (1U | USB_EP_DIR_OUT)
#define CDC_DATA_EPADDR_IN      (2U | USB_EP_DIR_IN)

#define CDC_DATA_EP_SIZE_OUT    64U
#define CDC_DATA_EP_SIZE_IN     64U

#define CDC_PROTOCOL USB_PROTOCOL_CDC_NONE

enum {
    ITF_NUM_CDC_CTRL,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

typedef struct __attribute__((packed)) {
    usb_desc_config_t        config;
    usb_desc_iad_t           com_iad;
    usb_desc_itf_t           com_itf;
    usb_cdc_desc_header_t    cdc_header;
    usb_cdc_desc_call_mgmt_t cdc_call_mgmt;
    usb_cdc_desc_acm_t       cdc_acm;
    usb_cdc_desc_union_t     cdc_union;
    usb_desc_endpoint_t      cdc_notif_ep;
    usb_desc_itf_t           cdc_data_itf;
    usb_desc_endpoint_t      cdc_data_ep_out;
    usb_desc_endpoint_t      cdc_data_ep_in;
} cdc_desc_config_t;

static const usb_desc_device_t CDC_DESC_DEVICE = {
    .bLength                = sizeof(usb_desc_device_t),
    .bDescriptorType        = USB_DTYPE_DEVICE,
    .bcdUSB                 = USB_BCD_VERSION_2_0,
    .bDeviceClass           = USB_CLASS_MISC,
    .bDeviceSubClass        = USB_SUBCLASS_IAD,
    .bDeviceProtocol        = USB_PROTOCOL_IAD,
    .bMaxPacketSize0        = CDC_CTRL_EP_SIZE,
    .idVendor               = 0xCafe,
    .idProduct              = 0x4005,
    .bcdDevice              = 0x0100,
    .iManufacturer          = 1,
    .iProduct               = 2,
    .iSerialNumber          = 3,
    .bNumConfigurations     = 1
};

static const cdc_desc_config_t CDC_DESC_CONFIG = {
    .config = {
        .bLength                = sizeof(usb_desc_config_t),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(cdc_desc_config_t),
        .bNumInterfaces         = ITF_NUM_TOTAL,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = USB_ATTR_RESERVED | USB_ATTR_SELF_POWERED,
        .bMaxPower              = 50, // 100mA
    },
    .com_iad = {
        .bLength                = sizeof(usb_desc_iad_t),
        .bDescriptorType        = USB_DTYPE_IAD,
        .bFirstInterface        = ITF_NUM_CDC_CTRL,
        .bInterfaceCount        = 2,
        .bFunctionClass         = USB_CLASS_CDC,
        .bFunctionSubClass      = USB_SUBCLASS_CDC_ABSTRACT_CONTROL,
        .bFunctionProtocol      = CDC_PROTOCOL,
        .iFunction              = 0
    },
    .com_itf = {
        .bLength                = sizeof(usb_desc_itf_t),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = ITF_NUM_CDC_CTRL,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_CDC,
        .bInterfaceSubClass     = USB_SUBCLASS_CDC_ABSTRACT_CONTROL,
        .bInterfaceProtocol     = CDC_PROTOCOL,
        .iInterface             = 0
    },
    .cdc_header = {
        .bFunctionLength        = sizeof(usb_cdc_desc_header_t),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_HEADER,
        .bcdCDC                 = 0x0110
    },
    .cdc_call_mgmt = {
        .bFunctionLength        = sizeof(usb_cdc_desc_call_mgmt_t),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_CALL_MGMT,
        .bmCapabilities         = 0x00,
        .bDataInterface         = ITF_NUM_CDC_DATA
    },
    .cdc_acm = {
        .bFunctionLength        = sizeof(usb_cdc_desc_acm_t),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_ACM,
        .bmCapabilities         = 0
    },
    .cdc_union = {
        .bFunctionLength        = sizeof(usb_cdc_desc_union_t),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_UNION,
        .bMasterInterface0      = ITF_NUM_CDC_CTRL,
        .bSlaveInterface0       = ITF_NUM_CDC_DATA
    },
    .cdc_notif_ep = {
        .bLength                = sizeof(usb_desc_endpoint_t),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_NOTIF_EPADDR_IN,
        .bmAttributes           = USB_EP_TYPE_INTERRUPT,
        .wMaxPacketSize         = CDC_NOTIF_EP_SIZE_IN,
        .bInterval              = 0xFF
    },
    .cdc_data_itf = {
        .bLength                = sizeof(usb_desc_itf_t),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = ITF_NUM_CDC_DATA,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_CDC_DATA,
        .bInterfaceSubClass     = USB_SUBCLASS_NONE,
        .bInterfaceProtocol     = USB_PROTOCOL_NONE,
        .iInterface             = 0
    },
    .cdc_data_ep_out = {
        .bLength                = sizeof(usb_desc_endpoint_t),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_DATA_EPADDR_OUT,
        .bmAttributes           = USB_EP_TYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_EP_SIZE_OUT,
        .bInterval              = 1
    },
    .cdc_data_ep_in = {
        .bLength                = sizeof(usb_desc_endpoint_t),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_DATA_EPADDR_IN,
        .bmAttributes           = USB_EP_TYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_EP_SIZE_IN,
        .bInterval              = 1
    }
};

static const usb_desc_string_t CDC_DESC_STR_LANGUAGE     = USB_ARRAY_DESC(0x0409);
static const usb_desc_string_t CDC_DESC_STR_MANUFACTURER = USB_STRING_DESC("WiredOpposite");
static const usb_desc_string_t CDC_DESC_STR_PRODUCT      = USB_STRING_DESC("CDC Device Example");
static const usb_desc_string_t* CDC_DESC_STR[] = {
    &CDC_DESC_STR_LANGUAGE,
    &CDC_DESC_STR_MANUFACTURER,
    &CDC_DESC_STR_PRODUCT
};

#ifdef __cplusplus
}
#endif