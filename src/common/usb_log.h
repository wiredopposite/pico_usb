#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef USB_LOG_LEVEL
#define USB_LOG_LEVEL 0
#endif

#ifndef USB_LOG_ENABLE
#define USB_LOG_ENABLE 0
#endif

#if USB_LOG_ENABLE
#include <stdio.h>
#define usb_log_l(level, ...) \
    do { \
        if (level <= USB_LOG_LEVEL) { \
            printf(__VA_ARGS__); \
        } \
    } while (0)
#define usb_log_l_hex(level, data, len) \
    do { \
        if (level <= USB_LOG_LEVEL) { \
            for (int i = 0; i < len; i++) { \
                printf(" %02x", ((uint8_t*)data)[i]); \
            } \
            printf("\n"); \
        } \
    } while (0)
#else
#define usb_log_l(level, ...)
#define usb_log_l_hex(level, data, len)
#endif

#define usb_loge(...)              usb_log_l(0, __VA_ARGS__)
#define usb_logi(...)              usb_log_l(1, __VA_ARGS__)
#define usb_logd(...)              usb_log_l(2, __VA_ARGS__)
#define usb_logv(...)              usb_log_l(3, __VA_ARGS__)

#define usb_loge_hex(...)          usb_log_l_hex(0, __VA_ARGS__)
#define usb_logi_hex(...)          usb_log_l_hex(1, __VA_ARGS__)
#define usb_logd_hex(...)          usb_log_l_hex(2, __VA_ARGS__)
#define usb_logv_hex(...)          usb_log_l_hex(3, __VA_ARGS__)

#ifdef __cplusplus
}
#endif