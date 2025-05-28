#include "usbd/usbd.h"
#if USBD_DEVICES_MAX

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <hardware/structs/usb.h>
#include <hardware/regs/usb.h>
#include <hardware/regs/resets.h>
#include <hardware/resets.h>
#include "common/usb_log.h"
#include "common/usb_def.h"
#include "common/usb_util.h"
#include "usbd/usbd.h"
#include "usbd/dcd/dcd.h"

#if (USBD_ENDPOINTS_MAX > USB_NUM_ENDPOINTS)
#error "Error: USBD_ENDPOINTS_MAX is greater than 16, not supported!"
#endif

#define usb_hw_set                  ((usb_hw_t*)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear                ((usb_hw_t*)hw_clear_alias_untyped(usb_hw))

#define USB_INTS_ERROR_Msk          (USB_INTS_ERROR_CRC_BITS        | \
                                    USB_INTS_ERROR_BIT_STUFF_BITS   | \
                                    USB_INTS_ERROR_RX_OVERFLOW_BITS | \
                                    USB_INTS_ERROR_RX_TIMEOUT_BITS  | \
                                    USB_INTS_ERROR_DATA_SEQ_BITS)

#define USB_SIE_STATUS_ERROR_Msk    (USB_SIE_STATUS_CRC_ERROR_BITS      | \
                                    USB_SIE_STATUS_BIT_STUFF_ERROR_BITS | \
                                    USB_SIE_STATUS_RX_OVERFLOW_BITS     | \
                                    USB_SIE_STATUS_RX_TIMEOUT_BITS      | \
                                    USB_SIE_STATUS_DATA_SEQ_ERROR_BITS)

typedef struct {
    bool     alloc;
    uint8_t  eptype;
    uint16_t epsize;
    uint8_t  pid;
    uint32_t buffer_offset; 
    bool     is_setup;
} hw_endpoint_t;

static hw_endpoint_t hw_endpoint[USB_NUM_ENDPOINTS * 2] = {0};
static uint32_t next_buffer_offset = 0;

static inline hw_endpoint_t* HW_EP(uint8_t epaddr) {
    uint8_t epnum = USB_EP_NUM(epaddr);
    if (epnum >= USB_NUM_ENDPOINTS) {
        return NULL;
    }
    return &hw_endpoint[epnum * 2 + (epaddr & USB_EP_DIR_IN ? 0U : 1U)];
}

static inline io_rw_32* EP_CTRL(uint8_t epaddr) {
    uint8_t epnum = USB_EP_NUM(epaddr);
    if (epnum >= USB_NUM_ENDPOINTS || epnum == 0) {
        return NULL;
    }
    if (epaddr & USB_EP_DIR_IN) {
        return &usb_dpram->ep_ctrl[epnum - 1].in;
    }
    return &usb_dpram->ep_ctrl[epnum - 1].out;
}

static inline io_rw_32* EP_BUF_CTRL(uint8_t epaddr) {
    uint8_t epnum = USB_EP_NUM(epaddr);
    if (epnum >= USB_NUM_ENDPOINTS) {
        return NULL;
    }
    if (epaddr & USB_EP_DIR_IN) {
        return &usb_dpram->ep_buf_ctrl[epnum].in;
    }
    return &usb_dpram->ep_buf_ctrl[epnum].out;
}

static inline uint8_t* EP_BUF(uint8_t epaddr) {
    uint8_t epnum = USB_EP_NUM(epaddr);
    if (epnum >= USB_NUM_ENDPOINTS) {
        return NULL;
    }
    if (epnum == 0) {
        return usb_dpram->ep0_buf_a;
    }
    hw_endpoint_t *ep = HW_EP(epaddr);
    if (ep == NULL || !ep->alloc) {
        return NULL;
    }
    return &usb_dpram->epx_data[ep->buffer_offset];
}

static void rearm_ep_out(uint8_t epaddr) {
    if (epaddr & USB_EP_DIR_IN) {
        return;
    }
    hw_endpoint_t* ep = HW_EP(epaddr);
    if (ep == NULL) {
        return;
    }
    uint32_t buf_ctrl = (ep->epsize | 
                        USB_BUF_CTRL_AVAIL |
                        (ep->pid 
                            ? USB_BUF_CTRL_DATA1_PID 
                            : USB_BUF_CTRL_DATA0_PID));
    *EP_BUF_CTRL(epaddr) = buf_ctrl;
    busy_wait_at_least_cycles(12);
}

static bool pico_usbd_init(void) {
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    memset(usb_dpram, 0, sizeof(*usb_dpram));

    usb_hw->muxing =    USB_USB_MUXING_TO_PHY_BITS | 
                        USB_USB_MUXING_SOFTCON_BITS;
    usb_hw->pwr =       USB_USB_PWR_VBUS_DETECT_BITS | 
                        USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->sie_ctrl =  USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    usb_hw->inte =      USB_INTE_BUS_RESET_BITS |
                        USB_INTE_SETUP_REQ_BITS | 
                        USB_INTE_BUFF_STATUS_BITS | 
                        USB_INTE_DEV_SUSPEND_BITS | 
                        USB_INTE_DEV_RESUME_FROM_HOST_BITS;
    return true;
}

static void pico_usbd_connect(bool connect) {
    if (connect) {
        usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
        usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
    } else {
        usb_hw_clear->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
        usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
    }
}

static void pico_usbd_deinit(void) {
    pico_usbd_connect(false);
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    memset(usb_dpram, 0, sizeof(*usb_dpram));
    memset(hw_endpoint, 0, sizeof(hw_endpoint));
    next_buffer_offset = 0;
}

static void pico_usbd_ep_set_stall(uint8_t dport, uint8_t epaddr, bool stall) {
    hw_endpoint_t *ep = HW_EP(epaddr);
    if (ep == NULL) {
        return;
    }
    if (stall) {
        if (USB_EP_NUM(epaddr) == 0) {
            uint32_t ep_stall = (epaddr & USB_EP_DIR_IN) ? 
                USB_EP_STALL_ARM_EP0_IN_BITS : USB_EP_STALL_ARM_EP0_OUT_BITS;
            usb_hw_set->ep_stall_arm = ep_stall;
        }
        uint32_t buf_ctrl = *EP_BUF_CTRL(epaddr);
        buf_ctrl &= ~USB_BUF_CTRL_AVAIL;
        buf_ctrl |= USB_BUF_CTRL_STALL;
        *EP_BUF_CTRL(epaddr) = buf_ctrl;
    } else {
        if (USB_EP_NUM(epaddr) != 0) {
            *EP_BUF_CTRL(epaddr) &= ~USB_BUF_CTRL_STALL;
        }
    }
    ep->pid = 0;
    usb_logv("USB %s STALL EP: %02X\n", stall ? "set" : "clear", epaddr);
}

static bool pico_usbd_ep_is_stalled(uint8_t dport, uint8_t epaddr) {
    uint8_t epnum = USB_EP_NUM(epaddr);
    if (epnum == 0 || epnum >= USB_NUM_ENDPOINTS) {
        return false;
    }
    return (*EP_BUF_CTRL(epaddr) & USB_BUF_CTRL_STALL);
}

static void pico_usbd_set_addr(uint8_t dport, uint8_t daddr) {
    usb_hw->dev_addr_ctrl = daddr;
    usb_logv("USB set address %d\n", daddr);
}

static bool pico_usbd_ep_config(uint8_t dport, uint8_t epaddr, uint8_t eptype, uint16_t epsize) {
    hw_endpoint_t *ep = HW_EP(epaddr);
    if ((eptype == USB_EP_TYPE_ISOCHRONUS) || (ep == NULL)) {
        return false;
    }
    memset(ep, 0, sizeof(hw_endpoint_t));
    ep->eptype = eptype;
    uintptr_t buf_ptr = 0;
    if (ep->eptype == USB_EP_TYPE_CONTROL) {
        ep->epsize = epsize;
    } else {
        ep->epsize = ((epsize + 63) / 64) * 64; // Round up to nearest 64 bytes
        if (ep->eptype & USB_EP_TYPE_BULK) {
            ep->epsize *= 2;
        }
        if ((next_buffer_offset + ep->epsize) > sizeof(usb_dpram->epx_data)) {
            return false; // Not enough space in DPRAM
        }
        ep->buffer_offset = next_buffer_offset;
        next_buffer_offset += ep->epsize;
        uintptr_t offset = 
            (uintptr_t)(offsetof(usb_device_dpram_t, epx_data) + ep->buffer_offset);
        const uint32_t ep_ctrl =    (EP_CTRL_ENABLE_BITS | 
                                    EP_CTRL_INTERRUPT_PER_BUFFER |
                                    ((uint)ep->eptype << EP_CTRL_BUFFER_TYPE_LSB) | 
                                    offset);
        *EP_CTRL(epaddr) = ep_ctrl;
        if (USB_EP_DIR(epaddr) == USB_EP_DIR_OUT) {
            rearm_ep_out(epaddr);
        }
    }
    ep->alloc = true;
    usb_logv("USB EP %02X configured: %s, size %d\n", 
        epaddr, (ep->eptype == USB_EP_TYPE_CONTROL) ? "control" : "other", ep->epsize);
    return true;
}

static void pico_usbd_ep_deconfig(uint8_t dport, uint8_t epaddr) {
    hw_endpoint_t *ep = HW_EP(epaddr);
    if (ep == NULL) {
        return;
    }
    if (USB_EP_NUM(epaddr) != 0) {
        *EP_CTRL(epaddr) = 0;
        *EP_BUF_CTRL(epaddr) = 0;
    }
    memset(ep, 0, sizeof(hw_endpoint_t));
}

static int32_t pico_usbd_ep_read_setup(uint8_t dport, uint8_t* buf) {
    int32_t ret = -1;
    hw_endpoint_t* ep = HW_EP(0 | USB_EP_DIR_OUT);
    if (ep == NULL) {
        return ret;
    }
    usb_logv("USB read SETUP\n");
    if (ep->is_setup) {
        for (uint16_t i = 0; i < sizeof(usb_dpram->setup_packet); i++) {
            buf[i] = usb_dpram->setup_packet[i];
        }
        ep->is_setup = false;
        ret = sizeof(usb_dpram->setup_packet);
    }
    ep->pid ^= 1;
    rearm_ep_out(0 | USB_EP_DIR_OUT);
    return ret;
}

static int32_t pico_usbd_ep_read(uint8_t dport, uint8_t epaddr, uint8_t* buf, uint16_t blen) {
    if (epaddr & USB_EP_DIR_IN) {
        usb_loge("Error: EP %02X is OUT, cannot read from it\n", epaddr);
        return -1;
    }
    hw_endpoint_t* ep = HW_EP(epaddr);
    if (ep == NULL) {
        return -1;
    }
    usb_logv("USB read EP %02X, len: %d\n", epaddr, blen);
    uint8_t* ep_buf = EP_BUF(epaddr);
    uint32_t len = *EP_BUF_CTRL(epaddr) & USB_BUF_CTRL_LEN_MASK;
    blen = MIN(len, blen);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = ep_buf[i];
    }
    ep->pid ^= 1;
    rearm_ep_out(epaddr);
    return blen;
}

static int32_t pico_usbd_ep_write(uint8_t dport, uint8_t epaddr, const uint8_t* buf, uint16_t blen) {
    if (!(epaddr & USB_EP_DIR_IN)) {
        usb_loge("Error: EP %02X is IN, cannot write to it\n", epaddr);
        return -1;
    }
    hw_endpoint_t* ep = HW_EP(epaddr);
    if (ep == NULL) {
        usb_loge("Error: EP %02X not configured\n", epaddr);
        return -1;
    }
    usb_logv("USB write EP %02X, len: %d\n", epaddr, blen);
    blen = MIN(blen, ep->epsize);
    uint8_t* ep_buf = EP_BUF(epaddr);
    for (uint16_t i = 0; i < blen; i++) {
        ep_buf[i] = buf[i];
    }
    *EP_BUF_CTRL(epaddr) =  (blen | 
                            USB_BUF_CTRL_AVAIL |
                            USB_BUF_CTRL_FULL  |
                            ((ep->pid == 1)
                                ? USB_BUF_CTRL_DATA1_PID 
                                : USB_BUF_CTRL_DATA0_PID));
    ep->pid ^= 1;
    return blen;
}

static void pico_usbd_ep_xfer_abort(uint8_t dport, uint8_t epaddr) {
    hw_endpoint_t* ep = HW_EP(epaddr);
    if (ep == NULL) {
        return;
    }
    uint32_t abort = (USB_EP_NUM(epaddr) << 1U) | 
                     ((epaddr & USB_EP_DIR_IN) ? 0U : 1U);
    if (rp2040_chip_version() >= 2) {
        usb_hw_set->abort = abort;
        while ((usb_hw->abort_done & abort) != abort) {
            tight_loop_contents();
        }
    }
    uint32_t buf_ctrl = USB_BUF_CTRL_SEL; // reset to buffer 0
    if ((epaddr & USB_EP_DIR_IN) && (USB_EP_NUM(epaddr) == 0)) {
        ep->pid = 1;
    }
    buf_ctrl |= (ep->pid) ? USB_BUF_CTRL_DATA1_PID : 0;
    *EP_BUF_CTRL(epaddr) = buf_ctrl;
    if (rp2040_chip_version() >= 2) {
        usb_hw_clear->abort_done = abort;
        usb_hw_clear->abort = abort;
    }
    printf("Aborted EP %02X transfer\n", epaddr);
}

static bool pico_usbd_ep_ready(uint8_t dport, uint8_t epaddr) {
    io_rw_32* buf_ctrl = EP_BUF_CTRL(epaddr);
    if (buf_ctrl != NULL) {
        if (epaddr & USB_EP_DIR_IN) {
            return !(*buf_ctrl & USB_BUF_CTRL_FULL);
        } else {
            return (*buf_ctrl & USB_BUF_CTRL_AVAIL);
        }
    }
    return true;
}

static uint16_t pico_usbd_get_frame(void) {
    return usb_hw->sof_rd & USB_SOF_RD_BITS;
}

static bool pico_usbd_task(uint8_t dport, uint32_t* event_mask, uint32_t* ep_mask) {
    uint32_t ints = usb_hw->ints;
    if (ints & USB_INTS_BUS_RESET_BITS) {
        *event_mask |= USBD_EVENT_RESET;
        usb_hw->dev_addr_ctrl = 0;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        for (uint8_t i = 0; i < USB_NUM_ENDPOINTS; i++) {
            pico_usbd_ep_deconfig(dport, i);
        }
        next_buffer_offset = 0;
#if PICO_RP2040_USB_DEVICE_ENUMERATION_FIX
        if (usb_hw->sie_ctrl & USB_SIE_CTRL_PULLUP_EN_BITS) {
            rp2040_usb_device_enumeration_fix();
        }
#endif
    }
    if (ints & USB_INTS_SETUP_REQ_BITS) {
        *event_mask |= USBD_EVENT_SETUP;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        *EP_BUF_CTRL(0 | USB_EP_DIR_IN) = 0;
        hw_endpoint_t* ep_out = HW_EP(0 | USB_EP_DIR_OUT);
        hw_endpoint_t* ep_in = HW_EP(0 | USB_EP_DIR_IN);
        ep_in->pid = 1;
        ep_out->pid = 0;
        if (!pico_usbd_ep_ready(dport, 0 | USB_EP_DIR_IN)) {
            /* We have a stale IN xfer pending */
            pico_usbd_ep_xfer_abort(dport, 0 | USB_EP_DIR_IN);
        }
        ep_out->is_setup = true;
    }
    if (ints & USB_INTS_BUFF_STATUS_BITS) {
        *event_mask |= USBD_EVENT_EP_CMPLT;
        *ep_mask = usb_hw->buf_status;
        usb_hw_clear->buf_status = *ep_mask;
    }
    if (ints & USB_INTS_DEV_SUSPEND_BITS) {
        *event_mask |= USBD_EVENT_SUSPEND;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SUSPENDED_BITS;
    }
    if (ints & USB_INTS_DEV_RESUME_FROM_HOST_BITS) {
        *event_mask |= USBD_EVENT_WAKEUP;
        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
    }
    if (ints & USB_INTS_ERROR_Msk) {
        *event_mask |= USBD_EVENT_ERROR;
        usb_hw_clear->sie_status = usb_hw->sie_status & USB_SIE_STATUS_ERROR_Msk;
    }
    return (*event_mask != 0);
}

const dcd_driver_t DCD_DRIVER_PICO = {
    .init = pico_usbd_init,
    .deinit = pico_usbd_deinit,
    .connect = pico_usbd_connect,
    .set_address = pico_usbd_set_addr,
    .ep_open = pico_usbd_ep_config,
    .ep_close = pico_usbd_ep_deconfig,
    .ep_xfer_abort = pico_usbd_ep_xfer_abort,
    .ep_read_setup = pico_usbd_ep_read_setup,
    .ep_read = pico_usbd_ep_read,
    .ep_write = pico_usbd_ep_write,
    .ep_set_stall = pico_usbd_ep_set_stall,
    .ep_stalled = pico_usbd_ep_is_stalled,
    .ep_ready = pico_usbd_ep_ready,
    .get_frame = pico_usbd_get_frame,
    .task = pico_usbd_task,
};

#pragma GCC pop_options

#endif /* USBD_DEVICES_MAX */