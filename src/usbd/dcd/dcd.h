#pragma once

#include "usbd/usbd.h"
#if USBD_DEVICES_MAX

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*dcd_init)(void);
typedef void (*dcd_deinit)(void);
typedef void (*dcd_connect)(void);
typedef bool (*dcd_ep_open)(uint8_t dport, uint8_t epaddr, uint8_t epattr, uint16_t epsize);
typedef void (*dcd_ep_close)(uint8_t dport, uint8_t epaddr);
typedef void (*dcd_set_address)(uint8_t dport, uint8_t daddr);
typedef void (*dcd_ep_xfer_abort)(uint8_t dport, uint8_t epaddr);
typedef int32_t (*dcd_ep_read_setup)(uint8_t dport, uint8_t* buffer);
typedef int32_t (*dcd_ep_read)(uint8_t dport, uint8_t epaddr, uint8_t* buffer, uint16_t len);
typedef int32_t (*dcd_ep_write)(uint8_t dport, uint8_t epaddr, const uint8_t* buffer, uint16_t len);
typedef void (*dcd_ep_set_stall)(uint8_t dport, uint8_t epaddr, bool stall);
typedef bool (*dcd_ep_stalled)(uint8_t dport, uint8_t epaddr);
typedef bool (*dcd_ep_ready)(uint8_t dport, uint8_t epaddr);
typedef uint16_t (*dcd_get_frame)(void);
typedef bool (*dcd_task)(uint8_t dport, uint32_t* event_mask, uint32_t* ep_mask);

typedef struct dcd_driver_ {
    dcd_init          init;
    dcd_deinit        deinit;
    dcd_connect       connect;
    dcd_set_address   set_address;
    dcd_ep_open       ep_open;
    dcd_ep_close      ep_close;
    dcd_ep_xfer_abort ep_xfer_abort;
    dcd_ep_read_setup ep_read_setup;
    dcd_ep_read       ep_read;
    dcd_ep_write      ep_write;
    dcd_ep_set_stall  ep_set_stall;
    dcd_ep_stalled    ep_stalled;
    dcd_ep_ready      ep_ready;
    dcd_get_frame     get_frame;
    dcd_task          task;
} dcd_driver_t;

extern const dcd_driver_t DCD_DRIVER_PICO;
extern const dcd_driver_t DCD_DRIVER_PIO;

#ifdef __cplusplus
}
#endif

#endif /* USBD_DEVICES_MAX */