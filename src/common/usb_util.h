#pragma once

#include <stdint.h>
#include <stddef.h>
#include "usb_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MIN
#define MIN(a,b)                    (((a) < (b)) ? (a) : (b))
#endif

#define ARRAY_SIZE(arr)             (sizeof(arr) / sizeof((arr)[0]))

#define USB_EP_NUM(epaddr)          ((epaddr) & 0x0F)
#define USB_EP_DIR(epaddr)          ((epaddr) & USB_EP_DIR_IN)
#define USB_EPADDR_FROM_IDX(idx)    ((idx >> 1U) | ((idx & 1U) ? USB_EP_DIR_OUT : USB_EP_DIR_IN))
#define USB_DESC_TYPE(wValue)       ((uint8_t)((wValue) >> 8U))
#define USB_DESC_INDEX(wValue)      ((uint8_t)((wValue) & 0xFFU))

#define __CAT(x,y)                  x ## y
#define CAT(x,y)                    __CAT(x,y)

/* Creates usb_desc_string_t from array */
#define USB_ARRAY_DESC(...) {                               \
        .bLength = 2 + sizeof((uint16_t[]){__VA_ARGS__}),   \
        .bDescriptorType = USB_DTYPE_STRING,                \
        .wString = {__VA_ARGS__}                            \
    }

#ifdef __cplusplus
/* Creates usb_desc_string_t from string */
#define USB_STRING_DESC(s) {                        \
    .bLength = 2 + sizeof(u##s) - sizeof(char16_t), \
    .bDescriptorType = USB_DTYPE_STRING,            \
    .wString = {(const uint16_t&)(u##s)[0]}         \
}
#else
/* Creates usb_desc_string_t from string */
#define USB_STRING_DESC(s) {                \
        .bLength = sizeof(CAT(u,s)),        \
        .bDescriptorType = USB_DTYPE_STRING,\
        .wString = {CAT(u,s)}               \
    }
#endif

/**
 * @brief Get an interface's class descriptor from a configuration descriptor.
 * 
 * This function searches for the specified interface number within a configuration
 * descriptor and returns a pointer to the class descriptor of that interface.
 * 
 * @param itf_num The interface number to search for.
 * @param desc_config Pointer to the configuration descriptor.
 * 
 * @return Pointer to the interface class descriptor, or NULL if not found.
 */
static const uint8_t* usbd_get_itf_class_desc(uint8_t itf_num, const uint8_t* desc_config) {
    if ((desc_config == NULL) ||
        (((const usb_desc_header_t*)desc_config)->bDescriptorType != USB_DTYPE_CONFIGURATION)) {
        return NULL;
    }

    const uint8_t* desc_p = desc_config;
    const uint8_t* end_p = desc_p + ((const usb_desc_config_t*)desc_p)->wTotalLength;

    while (desc_p < end_p) {
        const usb_desc_header_t* desc_hdr = (const usb_desc_header_t*)desc_p;
        if (desc_hdr->bLength == 0) {
            break;
        }
        if (desc_hdr->bDescriptorType == USB_DTYPE_INTERFACE) {
            const usb_desc_itf_t* itf_desc = (const usb_desc_itf_t*)desc_p;
            if ((itf_desc->bInterfaceNumber == itf_num)) {
                if ((desc_p + itf_desc->bLength) > end_p) {
                    return NULL;
                }
                return desc_p + itf_desc->bLength;
            }
        }
        desc_p += desc_hdr->bLength;
    }
    return NULL;
}

#ifdef __cplusplus
}
#endif