#include "usbd/usbd.h"
#if USBD_DEVICES_MAX

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <cstdint>
#include <cstdbool>
#include <cstring>
#include <algorithm>

#include <hardware/dma.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <hardware/irq.h>
#include <pico/sync.h>

#include "usb_crc.h"
#include "usb_rx.pio.h"
#include "usb_tx.pio.h"

#include "common/usb_def.h"
#include "common/usb_util.h"
#include "common/usb_log.h"
#include "usbd/usbd.h"
#include "usbd/dcd/dcd.h"

#ifndef USB_PIO_LOG_ENABLE
#define USB_PIO_LOG_ENABLE 0
#endif

/* Logging here will mess with timing */
#if USB_PIO_LOG_ENABLE
#define pusb_loge(...) usb_loge(__VA_ARGS__)
#define pusb_logi(...) usb_logi(__VA_ARGS__)
#define pusb_logd(...) usb_logd(__VA_ARGS__)
#define pusb_logv(...) usb_logv(__VA_ARGS__)
#define pusb_loge_hex(...) usb_loge_hex(__VA_ARGS__)
#define pusb_logi_hex(...) usb_logi_hex(__VA_ARGS__)
#define pusb_logd_hex(...) usb_logd_hex(__VA_ARGS__)
#define pusb_logv_hex(...) usb_logv_hex(__VA_ARGS__)
#else
#define pusb_loge(...)
#define pusb_logi(...)
#define pusb_logd(...)
#define pusb_logv(...)
#define pusb_loge_hex(...)
#define pusb_logi_hex(...)
#define pusb_logd_hex(...)
#define pusb_logv_hex(...)
#endif

typedef enum {
    PIO_USB_PINOUT_DPDM = 0,  // DM = DP+1
    PIO_USB_PINOUT_DMDP       // DM = DP-1
} pio_usb_pinout_t;

#ifndef PIO_USBD_DP_PIN
#define PIO_USBD_DP_PIN             0   /* Data+ pin number */
#endif  

#ifndef PIO_USBD_PINOUT 
#define PIO_USBD_PINOUT             PIO_USB_PINOUT_DPDM /* Pinout order */
#endif  

#ifndef PIO_USBD_PIO_TX 
#define PIO_USBD_PIO_TX             0   /* TX PIO index */
#endif  

#ifndef PIO_USBD_PIO_RX 
#define PIO_USBD_PIO_RX             1   /* RX PIO index */
#endif  

#ifndef PIO_USBD_SM_TX  
#define PIO_USBD_SM_TX              0   /* Default TX state machine index */
#endif  

#ifndef PIO_USBD_SM_RX  
#define PIO_USBD_SM_RX              0   /* Default RX state machine index */
#endif  

#ifndef PIO_USBD_SM_RX_EOP  
#define PIO_USBD_SM_RX_EOP          1   /* Default EOP state machine index */
#endif  

#ifndef PIO_USBD_DMA_CHAN_TX        
#define PIO_USBD_DMA_CHAN_TX        0   /* Default TX DMA channel */
#endif

#define PIO_USB_CLK_DIV_TX          ((float)48000000)
#define PIO_USB_CLK_DIV_RX          ((float)96000000)
#define PIO_USB_RX_TIMEOUT_US       8U
#define PIO_USBD_EP_BUF_SIZE        ((uint16_t)USBD_ENDPOINT_MAX_SIZE + 4U) /* Sync + PID + CRC = 4 */

#define PIO_USB_IRQ_TX_EOP_Msk      (1U << usb_tx_dpdm_IRQ_EOP)
#define PIO_USB_IRQ_TX_COMP_Msk     (1U << usb_tx_dpdm_IRQ_COMP)
#define PIO_USB_IRQ_TX_Msk          (PIO_USB_IRQ_TX_EOP_Msk | PIO_USB_IRQ_TX_COMP_Msk)
#define PIO_USB_IRQ_RX_COMP_Msk     (1U << IRQ_RX_EOP)
#define PIO_USB_IRQ_RX_Msk          ((1U << IRQ_RX_EOP)      | \
                                     (1U << IRQ_RX_BS_ERR)   | \
                                     (1U << IRQ_RX_START)    | \
                                     (1U << DECODER_TRIGGER))

#define PIO_USB_EP_MASK(epaddr)     (1 << ((USB_EP_NUM(epaddr) * 2) + (((epaddr) & USB_EP_DIR_IN) ? 0 : 1)))

typedef enum {
    PIO_USB_PORT_PIN_SE0        = 0b00,
    PIO_USB_PORT_PIN_FS_IDLE    = 0b01,
    PIO_USB_PORT_PIN_LS_IDLE    = 0b10,
    PIO_USB_PORT_PIN_SE1        = 0b11,
} usb_pin_status_t;

typedef enum : uint8_t {
    PIO_USB_SYNC        = 0x80,
    PIO_USB_PID_OUT     = 0xe1,
    PIO_USB_PID_IN      = 0x69,
    PIO_USB_PID_SOF     = 0xa5,
    PIO_USB_PID_SETUP   = 0x2d,
    PIO_USB_PID_DATA0   = 0xc3,
    PIO_USB_PID_DATA1   = 0x4b,
    PIO_USB_PID_ACK     = 0xd2,
    PIO_USB_PID_NAK     = 0x5a,
    PIO_USB_PID_STALL   = 0x1e,
    PIO_USB_PID_PRE     = 0x3c,
    PIO_USB_CRC16_PLACE = 0,
} usb_pid_t;

typedef union __attribute__((packed, aligned(4))) {
    uint8_t raw[4];
    struct {
        uint32_t sync  : 8;
        uint32_t pid   : 8;
        uint32_t daddr : 7;
        uint32_t epnum : 4;
        uint32_t crc   : 5;
    };
    struct {
        uint32_t sof_sync : 8;
        uint32_t sof_pid  : 8;
        uint32_t sof_num  : 11;
        uint32_t sof_crc  : 5;
    };
} usbd_token_t;
static_assert(sizeof(usbd_token_t) == 4, "usbd_token_t size mismatch");

typedef struct {
    volatile bool   busy{false};
    volatile bool   stalled{false};
    volatile bool   has_xfer{false};
    uint16_t        len{0};
    uint8_t         data[PIO_USBD_EP_BUF_SIZE] __attribute__((aligned(4))){0};
} ep_buf_t;

typedef struct {
    // uint8_t              crc5_lut{0};
    uint8_t             epaddr{0};
    uint8_t             epattr{0};
    uint16_t            epsize{0};
    volatile uint8_t    data_id{0};
    ep_buf_t            buf_in;
    ep_buf_t            buf_out;
} usbd_endpoint_t;

typedef struct {
    uint32_t device{0};
    uint32_t ep{0};
} pio_usbd_ints_t;

class usbd_device {
public:
    usbd_device() = default;
    ~usbd_device() = default;

    uint8_t             dport{0};
    volatile uint8_t    daddr{0};
    volatile uint8_t    daddr_pending{0};
    usbd_endpoint_t     ep[USBD_ENDPOINTS_MAX];
    uint8_t             setup_buf[8] __attribute__((aligned(4))){0};
    uint8_t             setup_len{0};
    volatile bool       has_setup{false};

    inline pio_usbd_ints_t load_clear_ints(void) {
        pio_usbd_ints_t ints = {0};
        ints.device = dev_ints;
        ints.ep = ep_ints;
        dev_ints = 0;
        ep_ints = 0;
        return ints;
    }

    inline void set_ints(uint32_t device, uint32_t ep) {    
        dev_ints |= device;
        ep_ints |= ep;
    }

    inline void store_ints(uint32_t device, uint32_t ep) {
        dev_ints = device;
        ep_ints = ep;
    }

private:
    volatile uint32_t dev_ints{0};
    volatile uint32_t ep_ints{0};
};

typedef struct {
    uint16_t div_int{0};
    uint8_t  div_frac{0};
} usbd_clk_div_t;

typedef struct {
    bool                alloc{false};
    bool                connected{false};
    usbd_device         dev[USBD_DEVICES_MAX];
    volatile bool       reset{false};
    volatile uint32_t   frame_num{0};
    
    uint8_t pin_dp{0};
    uint8_t pin_dm{0};
    PIO     pio_tx{nullptr};
    PIO     pio_rx{nullptr};
    uint    sm_tx{0};
    uint    sm_rx{0};
    uint    sm_eop{0};
    uint    offset_tx{0};
    uint    offset_rx{0};
    uint    offset_eop{0};
    uint    reset_rx_instr{0};
    uint    reset_rx_instr2{0};
    uint    rx_irq_num{0};
    uint    dma_chan_tx{0};

    const pio_program_t *fs_tx_program{nullptr};
    const pio_program_t *fs_tx_pre_program{nullptr};
    const pio_program_t *ls_tx_program{nullptr};

    usbd_clk_div_t clk_div_fs_tx;
    usbd_clk_div_t clk_div_fs_rx;
    usbd_clk_div_t clk_div_ls_tx;
    usbd_clk_div_t clk_div_ls_rx;

    // uint8_t temp_buf[256] __attribute__((aligned(4))) {0};
} usbd_rhport_t;

static const uint8_t REQ_DADDR[] = { 0x00, 0x05 };
static uint8_t NAK_PACKET[2]    __attribute__((aligned(4))) = { PIO_USB_SYNC, PIO_USB_PID_NAK };
static uint8_t ACK_PACKET[2]    __attribute__((aligned(4))) = { PIO_USB_SYNC, PIO_USB_PID_ACK };
static uint8_t STALL_PACKET[2]  __attribute__((aligned(4))) = { PIO_USB_SYNC, PIO_USB_PID_STALL };

static usbd_rhport_t usbd_rhport;
static const size_t pio_usbd_rhport_size = sizeof(usbd_rhport);

/* ---- Private ---- */

static usbd_endpoint_t* __no_inline_not_in_flash_func(get_endpoint)(uint8_t dport, uint8_t epaddr) {
    if (dport < USBD_DEVICES_MAX) {
        uint8_t epnum = USB_EP_NUM(epaddr);
        if (epnum < USBD_ENDPOINTS_MAX) {
            return &usbd_rhport.dev[dport].ep[epnum];
        }
    }
    return NULL;
}

static __always_inline usb_pin_status_t usbd_bus_get_line_state(usbd_rhport_t *rhport) {
    uint8_t dp = gpio_get(rhport->pin_dp) ? 0 : 1;
    uint8_t dm = gpio_get(rhport->pin_dm) ? 0 : 1;
    return (usb_pin_status_t)((dm << 1) | dp);
}

// static void usbd_update_daddr_crc(usbd_device* device, uint8_t daddr) {
//     uint16_t dat;
//     uint8_t crc;
//     device->daddr.store(daddr, std::memory_order_release);
//     // for (int epnum = 0; epnum < USBD_ENDPOINTS_MAX; epnum++) {
//     //     dat = (daddr) | (epnum << 7);
//     //     crc = calc_usb_crc5(dat);
//     //     device->ep[epnum].crc5_lut = (crc << 3) | ((epnum >> 1) & 0x07);
//     // }
// }

static __always_inline void usbd_restart_receiver(usbd_rhport_t* rhport) {
    pio_sm_exec(rhport->pio_rx, rhport->sm_rx, rhport->reset_rx_instr);
    pio_sm_exec(rhport->pio_rx, rhport->sm_rx, rhport->reset_rx_instr2);
    pio_sm_restart(rhport->pio_rx, rhport->sm_rx);
    rhport->pio_rx->irq = PIO_USB_IRQ_RX_Msk;
}

static void __no_inline_not_in_flash_func(usbd_bus_prepare_receive)(const usbd_rhport_t* rhport) {
    pio_sm_set_enabled(rhport->pio_rx, rhport->sm_rx, false);
    pio_sm_clear_fifos(rhport->pio_rx, rhport->sm_rx);
    pio_sm_restart(rhport->pio_rx, rhport->sm_rx);
    pio_sm_exec(rhport->pio_rx, rhport->sm_rx, rhport->reset_rx_instr);
    pio_sm_exec(rhport->pio_rx, rhport->sm_rx, rhport->reset_rx_instr2);
}

static uint8_t __no_inline_not_in_flash_func(usbd_get_handshake_blocking)(const usbd_rhport_t* rhport) {
    int16_t t = 240;
    int16_t idx = 0;
    uint8_t rx[2] = {0};
    while (t--) {
        if (!pio_sm_get_rx_fifo_level(rhport->pio_rx, rhport->sm_rx)) {
            continue;
        }
        rx[idx++] = pio_sm_get(rhport->pio_rx, rhport->sm_rx) >> 24;
        if (idx == 2) {
            break;
        }
    }
    // if (t > 0) {
    //     while ((rhport->pio_rx->irq & PIO_USB_IRQ_RX_COMP_Msk) == 0) {
    //         tight_loop_contents();
    //     }
    // }
    pio_sm_set_enabled(rhport->pio_rx, rhport->sm_rx, false);
    return rx[1];
}

static __always_inline void usbd_bus_start_receive(const usbd_rhport_t* rhport) {
    rhport->pio_rx->ctrl |= (1 << rhport->sm_rx);
    rhport->pio_rx->irq = PIO_USB_IRQ_RX_Msk;
}

static void __not_in_flash_func(usbd_in_xfer_blocking)(const usbd_rhport_t* rhport, 
                                                       const uint8_t* buffer, uint16_t len) {
    pio_sm_set_enabled(rhport->pio_tx, rhport->sm_tx, true);
    __dmb();
    dma_channel_transfer_from_buffer_now(rhport->dma_chan_tx, buffer, len);
    rhport->pio_tx->irq = PIO_USB_IRQ_TX_Msk;
    while ((rhport->pio_tx->irq & PIO_USB_IRQ_TX_Msk) == 0) {
        tight_loop_contents();
    }
}

// static void __no_inline_not_in_flash_func(usbd_send_handshake)(const usbd_rhport_t* rhport, uint8_t pid) {
//     uint8_t data[] = { PIO_USB_SYNC, pid };
//     usbd_in_xfer_blocking(rhport, data, sizeof(data));
// }

// static inline __force_inline bool pio_usb_bus_wait_for_rx_start(const pio_port_t* pp) {
//     // USB 2.0 specs: 7.1.19.1: handshake timeout
//     // Full-Speed (12 Mbps): 1 bit time = 1 / 12 MHz = 83.3 ns --> 16 bit times = 1.33 µs
//     // Low-Speed (1.5 Mbps): 1 bit time = 1 / 1.5 MHz = 666.7 ns --> 16 bit times = 10.67 µs
  
//     // We're starting the timing somewhere in the current microsecond so always assume the first one
//     // is less than a full microsecond. For example, a wait of 2 could actually be 1.1 microseconds.
//     // We will use 3 us (24 bit time) for Full speed and 12us (18 bit time) for Low speed.
//     uint32_t start = time_us_32();
//     uint32_t timeout = pp->low_speed ? 12 : 3;
//     while (get_time_us_32() - start <= timeout) {
//         if ((pp->pio_usb_rx->irq & IRQ_RX_START_MASK) != 0) {
//             return true;
//         }
//     }
//     return false;
// };

static int __no_inline_not_in_flash_func(usbd_get_packet)(usbd_rhport_t* rhport, uint8_t* buffer) {
    uint16_t crc = 0xffff;
    uint16_t crc_prev = 0xffff;
    uint16_t crc_prev2 = 0xffff;
    uint16_t crc_receive = 0xffff;
    uint16_t crc_receive_inverse = 0;
    bool crc_match = false;
    uint16_t idx = 0;
    
    /*  Timeout is seven microseconds. That is enough time to receive one byte at low speed.
        This is to detect packets without an EOP because the device was unplugged. */
    uint32_t start = time_us_32();
    while (((time_us_32() - start) <= PIO_USB_RX_TIMEOUT_US) && 
           ((rhport->pio_rx->irq & PIO_USB_IRQ_RX_COMP_Msk) == 0) &&
           (idx < PIO_USBD_EP_BUF_SIZE)) {
        if (pio_sm_get_rx_fifo_level(rhport->pio_rx, rhport->sm_rx)) {
            uint8_t data = pio_sm_get(rhport->pio_rx, rhport->sm_rx) >> 24;
            if ((buffer != nullptr) && (idx > 1)) {
                /* Skip the first 2 bytes, SYNC and PID */
                buffer[idx - 2] = data;
            }
            start = time_us_32(); /* Reset timeout when a byte is received */
        
            if (idx >= 2) {
                crc_prev2 = crc_prev;
                crc_prev = crc;
                crc = update_usb_crc16(crc, data);
                crc_receive = (crc_receive >> 8) | (data << 8);
                crc_receive_inverse = crc_receive ^ 0xffff;
                crc_match = (crc_receive_inverse == crc_prev2);
            }
            idx++;
        } 
    }
    if (buffer != nullptr) {
        if (idx >= 4 && crc_match) {
            return idx - 4;
        } else if (idx > 0) {
            pusb_logd("D: RX, CRC %s: ", crc_match ? "match" : "mismatch");
            pusb_logd_hex(buffer, idx);
        }
    }
    return -1;
}

static __always_inline void usbd_clear_ctrl_ep_stall(usbd_device* device) {
    device->ep[0].buf_in.stalled = false;
    device->ep[0].buf_out.stalled = false;
}

static void __no_inline_not_in_flash_func(usbd_handle_in_token)(usbd_rhport_t* rhport, 
                                                                usbd_device* device, 
                                                                usbd_endpoint_t* ep) {
    pio_sm_set_enabled(rhport->pio_rx, rhport->sm_rx, false);

    if (ep->buf_in.has_xfer) {
        ep->buf_in.busy = true;
        ep->buf_in.data[1] = 
            (ep->data_id == 0) ? PIO_USB_PID_DATA0 : PIO_USB_PID_DATA1;
        
        usbd_in_xfer_blocking(rhport, ep->buf_in.data, ep->buf_in.len);
        
        if ((device->daddr_pending != 0) && ((ep->buf_in.len - 4) == 0)) {
            /* This is a ZLP response to SET_ADDRESS */
            device->daddr = device->daddr_pending;
            device->daddr_pending = 0;
        }
        ep->data_id = (ep->data_id == 0) ? 1 : 0;
    } 
    else if (ep->buf_in.stalled) {
        usbd_in_xfer_blocking(rhport, STALL_PACKET, sizeof(STALL_PACKET));
        if (USB_EP_NUM(ep->epaddr) == 0) {
            usbd_clear_ctrl_ep_stall(device);
        }
    } 
    else {
        usbd_in_xfer_blocking(rhport, NAK_PACKET, sizeof(NAK_PACKET));
    }

    rhport->pio_rx->irq = PIO_USB_IRQ_RX_Msk;
    irq_clear(rhport->rx_irq_num);
    usbd_bus_start_receive(rhport);

    if (ep->buf_in.has_xfer) {
        /* Wait for ACK to our IN xfer */
        // usbd_get_handshake_blocking(rhport);
        usbd_bus_start_receive(rhport);
        irq_clear(rhport->rx_irq_num);

        ep->buf_in.has_xfer = false;
        ep->buf_in.busy = false;

        device->set_ints(
            USBD_EVENT_EP_CMPLT, 
            PIO_USB_EP_MASK(USB_EP_NUM(ep->epaddr) | USB_EP_DIR_IN)
        );
    }   
}

static void __no_inline_not_in_flash_func(usbd_handle_out_token)(usbd_rhport_t* rhport, 
                                                                 usbd_device* device, 
                                                                 usbd_endpoint_t* ep) {
    int len = -1;
    if (ep->buf_out.stalled) {
        usbd_in_xfer_blocking(rhport, STALL_PACKET, sizeof(STALL_PACKET));
        if (USB_EP_NUM(ep->epaddr) == 0) {
            usbd_clear_ctrl_ep_stall(device);
        }
    } else if (ep->buf_out.has_xfer) {
        usbd_in_xfer_blocking(rhport, NAK_PACKET, sizeof(NAK_PACKET));
    } else {
        len = usbd_get_packet(rhport, ep->buf_out.data);
        usbd_in_xfer_blocking(rhport, ACK_PACKET, sizeof(ACK_PACKET));
    }

    // pio_sm_clear_fifos(rhport->pio_rx, rhport->sm_rx);
    usbd_restart_receiver(rhport);
    irq_clear(rhport->rx_irq_num);

    if (len >= 0) {
        ep->buf_out.len = len;
        ep->buf_out.has_xfer = true;
        device->set_ints(
            USBD_EVENT_EP_CMPLT, 
            PIO_USB_EP_MASK(USB_EP_NUM(ep->epaddr) | USB_EP_DIR_OUT)
        );
    }
    usbd_bus_start_receive(rhport);
}

static void __no_inline_not_in_flash_func(usbd_handle_setup_token)(usbd_rhport_t* rhport, 
                                                                   usbd_device* device, 
                                                                   usbd_endpoint_t* ep) {
    int len = usbd_get_packet(rhport, (device->has_setup ? nullptr : device->setup_buf));
    /* ACK all SETUPs */
    usbd_in_xfer_blocking(rhport, ACK_PACKET, sizeof(ACK_PACKET));

    pio_sm_clear_fifos(rhport->pio_rx, rhport->sm_rx);
    usbd_restart_receiver(rhport);
    irq_clear(rhport->rx_irq_num);
    
    if (len >= 0) {
        ep->data_id = 1;
        ep->buf_out.stalled = false;
        device->setup_len = len;
        device->has_setup = true;
        if (std::memcmp(device->setup_buf, REQ_DADDR, sizeof(REQ_DADDR)) == 0) {
            /* In case the app isn't fast enough to set a new daddr */
            device->daddr_pending = device->setup_buf[2];
        } 
        usbd_bus_start_receive(rhport);
        device->set_ints(USBD_EVENT_SETUP, 0);
    } else {
        usbd_bus_start_receive(rhport);
    }
    if (len < 0) {
        pusb_loge("E: Setup read failed\n");
    }
}

static void __no_inline_not_in_flash_func(usbd_packet_handler)(void) {
    usbd_rhport.reset = false;
    usbd_token_t token = {0};
    uint8_t idx = 0;
    uint32_t start = time_us_32();
    while (((time_us_32() - start) <= 16) && 
            ((usbd_rhport.pio_rx->irq & PIO_USB_IRQ_RX_COMP_Msk) == 0) &&
            (idx < sizeof(usbd_token_t))) {
        if (pio_sm_get_rx_fifo_level(usbd_rhport.pio_rx, usbd_rhport.sm_rx)) {
            token.raw[idx++] = pio_sm_get(usbd_rhport.pio_rx, usbd_rhport.sm_rx) >> 24;
            start = time_us_32(); // reset timeout when a byte is received
            if ((idx == 1) && (token.raw[0] != PIO_USB_SYNC)) {
                idx = 0;
                continue;
            } else if ((idx == 2) && (token.raw[1] == PIO_USB_PID_SOF)) {
                /* This needs to happen very quickly */
                while ((usbd_rhport.pio_rx->irq & PIO_USB_IRQ_RX_COMP_Msk) == 0) {
                    tight_loop_contents();
                }
                pio_sm_clear_fifos(usbd_rhport.pio_rx, usbd_rhport.sm_rx);
                usbd_restart_receiver(&usbd_rhport);
                return;
            }
        }
    }
    if (idx != sizeof(usbd_token_t)) {
        /* Timed out, ignore. */
        pio_sm_clear_fifos(usbd_rhport.pio_rx, usbd_rhport.sm_rx);
        usbd_restart_receiver(&usbd_rhport);
        return;
    }
    usbd_restart_receiver(&usbd_rhport);

    usbd_device* device = nullptr;
    usbd_endpoint_t* ep = nullptr;
    if (token.epnum < USBD_ENDPOINTS_MAX) {
        for (uint8_t i = 0; i < USBD_DEVICES_MAX; i++) {
            if (usbd_rhport.dev[i].daddr == token.daddr) {
                device = &usbd_rhport.dev[i];
                ep = &usbd_rhport.dev[i].ep[token.epnum];
                break;
            }
        }
    }
    if (device == nullptr) {
        /*  Token addressed to an unconfigured device, 
            just read and return w/o handshake */
        if ((token.pid == PIO_USB_PID_OUT) || 
            (token.pid == PIO_USB_PID_SETUP)) {
            usbd_get_packet(&usbd_rhport, nullptr);
            // pio_sm_clear_fifos(usbd_rhport.pio_rx, usbd_rhport.sm_rx);
            usbd_restart_receiver(&usbd_rhport);
        }
        return;
    }
    switch (token.pid) {
    case PIO_USB_PID_IN:
        usbd_handle_in_token(&usbd_rhport, device, ep);
        break;
    case PIO_USB_PID_OUT:
        usbd_handle_out_token(&usbd_rhport, device, ep);
        break;
    case PIO_USB_PID_SETUP:
        usbd_handle_setup_token(&usbd_rhport, device, ep);
        break;
    default:
        pusb_logd("Unknown token: %02X\n", token.raw[1]);
        break;
    }
}

static __always_inline void add_pio_host_rx_program(PIO pio, const pio_program_t *program,
                                                    const pio_program_t *debug_program,
                                                    uint *offset, int debug_pin) {
    if (debug_pin < 0) {
        *offset = pio_add_program(pio, program);
    } else {
        *offset = pio_add_program(pio, debug_program);
    }
}

static __always_inline void reset_ep_buf(ep_buf_t* ep_buf) {
    ep_buf->has_xfer = false;
    ep_buf->stalled = false;
}

static void init_rhport(usbd_rhport_t* rhport) {
    rhport->alloc       = true;
    rhport->pio_tx      = (PIO_USBD_PIO_TX == 0) ? pio0 : pio1;
    rhport->pio_rx      = (PIO_USBD_PIO_RX == 0) ? pio0 : pio1;
    rhport->sm_tx       = PIO_USBD_SM_TX;
    rhport->sm_rx       = PIO_USBD_SM_RX;
    rhport->sm_eop      = PIO_USBD_SM_RX_EOP;
    rhport->dma_chan_tx = PIO_USBD_DMA_CHAN_TX;
    rhport->rx_irq_num  = (rhport->pio_rx == pio0)
                         ? PIO0_IRQ_0 : PIO1_IRQ_0;
    rhport->pin_dp      = PIO_USBD_DP_PIN;

    if (PIO_USBD_PINOUT == PIO_USB_PINOUT_DPDM) {
        rhport->pin_dm              = rhport->pin_dp + 1;
        rhport->fs_tx_program       = &usb_tx_dpdm_program;
        rhport->fs_tx_pre_program   = &usb_tx_pre_dpdm_program;
        rhport->ls_tx_program       = &usb_tx_dmdp_program;
    } else {
        rhport->pin_dm              = rhport->pin_dp - 1;
        rhport->fs_tx_program       = &usb_tx_dmdp_program;
        rhport->fs_tx_pre_program   = &usb_tx_pre_dmdp_program;
        rhport->ls_tx_program       = &usb_tx_dpdm_program;
    }

    for (uint8_t i = 0; i < USBD_DEVICES_MAX; i++) {
        rhport->dev[i].dport = i;
    }
}

static void init_dma(const usbd_rhport_t* rhport) {
    dma_claim_mask(1 << rhport->dma_chan_tx);
    dma_channel_config dma_cfg = 
        dma_channel_get_default_config(rhport->dma_chan_tx);
    channel_config_set_read_increment(&dma_cfg, true);
    channel_config_set_write_increment(&dma_cfg, false);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
    channel_config_set_dreq(
        &dma_cfg, 
        pio_get_dreq(rhport->pio_tx, rhport->sm_tx, true)
    );
    dma_channel_set_config(rhport->dma_chan_tx, &dma_cfg, false);
    dma_channel_set_write_addr(
        rhport->dma_chan_tx, 
        &rhport->pio_tx->txf[rhport->sm_tx], 
        false
    );
}

/* ---- Public ---- */

static void pio_usbd_connect(bool connect);

static bool pio_usbd_init(void) {
    usbd_rhport_t* rhport = &usbd_rhport;
    if (rhport->alloc) {
        return false; // RHPORT already allocated or invalid pinout
    }

    float cpu_freq = (float)clock_get_hz(clk_sys);
    if (cpu_freq != (240*1000*1000)) {
        pusb_loge("Error: CPU frequency must be 240MHz for PIO USB\n");
        return false;
    }

    //Init rhport struct
    init_rhport(rhport);

    //Init DMA
    init_dma(rhport);

    //Init PIO
    pio_sm_claim(rhport->pio_tx, rhport->sm_tx);
    pio_sm_claim(rhport->pio_rx, rhport->sm_rx);
    pio_sm_claim(rhport->pio_rx, rhport->sm_eop);

    rhport->offset_tx = pio_add_program(rhport->pio_tx, rhport->fs_tx_program);
    usb_tx_fs_program_init(
        rhport->pio_tx, 
        rhport->sm_tx, 
        rhport->offset_tx,
        rhport->pin_dp, 
        rhport->pin_dm
    );
    add_pio_host_rx_program(
        rhport->pio_rx, 
        &usb_nrzi_decoder_program,
        &usb_nrzi_decoder_debug_program, 
        &rhport->offset_rx,
        -1
    );
    usb_rx_fs_program_init(
        rhport->pio_rx, 
        rhport->sm_rx, 
        rhport->offset_rx, 
        rhport->pin_dp,
        rhport->pin_dm, 
        -1
    );
    rhport->reset_rx_instr = pio_encode_jmp(rhport->offset_rx);
    rhport->reset_rx_instr2 = pio_encode_set(pio_x, 0);
  
    add_pio_host_rx_program(
        rhport->pio_rx, 
        &usb_edge_detector_program,
        &usb_edge_detector_debug_program, 
        &rhport->offset_eop,
        -1
    );
    eop_detect_fs_program_init(
        rhport->pio_rx, 
        rhport->sm_eop, 
        rhport->offset_eop,
        rhport->pin_dp, 
        rhport->pin_dm, 
        true,
        -1
    );
  
    usb_tx_configure_pins(rhport->pio_tx, rhport->sm_tx, rhport->pin_dp, rhport->pin_dm);
  
    pio_sm_set_jmp_pin(rhport->pio_rx, rhport->sm_rx, rhport->pin_dp);
    pio_sm_set_jmp_pin(rhport->pio_rx, rhport->sm_eop, rhport->pin_dm);
    pio_sm_set_in_pins(rhport->pio_rx, rhport->sm_eop, rhport->pin_dp);

    gpio_set_slew_rate(rhport->pin_dp, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(rhport->pin_dm, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(rhport->pin_dp, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(rhport->pin_dm, GPIO_DRIVE_STRENGTH_12MA);

    pio_calculate_clkdiv_from_float(cpu_freq / PIO_USB_CLK_DIV_TX,
                                    &rhport->clk_div_fs_tx.div_int,
                                    &rhport->clk_div_fs_tx.div_frac);
    pio_calculate_clkdiv_from_float(cpu_freq / PIO_USB_CLK_DIV_RX,
                                    &rhport->clk_div_fs_rx.div_int,
                                    &rhport->clk_div_fs_rx.div_frac);

    pio_sm_set_clkdiv_int_frac(
        rhport->pio_tx, 
        rhport->sm_tx, 
        rhport->clk_div_fs_tx.div_int, 
        rhport->clk_div_fs_tx.div_frac
    );
  
    pio_sm_set_jmp_pin(rhport->pio_rx, rhport->sm_rx, rhport->pin_dp);
    pio_sm_set_clkdiv_int_frac(rhport->pio_rx, rhport->sm_rx, 1, 0);
  
    pio_sm_set_jmp_pin(rhport->pio_rx, rhport->sm_eop, rhport->pin_dm);
    pio_sm_set_in_pins(rhport->pio_rx, rhport->sm_eop, rhport->pin_dp);
    pio_sm_set_clkdiv_int_frac(
        rhport->pio_rx, 
        rhport->sm_eop, 
        rhport->clk_div_fs_rx.div_int, 
        rhport->clk_div_fs_rx.div_frac
    );


    pio_sm_set_enabled(rhport->pio_tx, rhport->sm_tx, true);
    usbd_bus_prepare_receive(rhport);
    rhport->pio_rx->ctrl |= (1 << rhport->sm_rx);
    rhport->pio_rx->irq |= PIO_USB_IRQ_RX_Msk;

    // enable the two PIO SMs that run USB RX
    // pio_sm_set_enabled(rhport->pio_rx,  rhport->sm_rx,  true);
    pio_sm_set_enabled(rhport->pio_rx,  rhport->sm_eop, true);

    pio_set_irqn_source_enabled(
        rhport->pio_rx, 
        0, 
        (pio_interrupt_source)(pis_interrupt0 + IRQ_RX_START),
        true
    );

    irq_set_exclusive_handler(rhport->rx_irq_num, usbd_packet_handler);
    irq_set_enabled(rhport->rx_irq_num, true);
    return true;
}

static void pio_usbd_deinit(void) {
    if (!usbd_rhport.connected) {
        return;
    }
    usbd_rhport.connected = false;
    irq_set_enabled(usbd_rhport.rx_irq_num, false);
    dma_channel_abort(usbd_rhport.dma_chan_tx);
    dma_channel_unclaim(usbd_rhport.dma_chan_tx);
    pio_sm_set_enabled(usbd_rhport.pio_tx, usbd_rhport.sm_tx, false);
    pio_sm_set_enabled(usbd_rhport.pio_rx, usbd_rhport.sm_rx, false);
    pio_sm_set_enabled(usbd_rhport.pio_rx, usbd_rhport.sm_eop, false);
    pio_sm_clear_fifos(usbd_rhport.pio_tx, usbd_rhport.sm_tx);
    pio_sm_clear_fifos(usbd_rhport.pio_rx, usbd_rhport.sm_rx);
    pio_sm_clear_fifos(usbd_rhport.pio_rx, usbd_rhport.sm_eop);
    pio_remove_program(usbd_rhport.pio_tx, usbd_rhport.fs_tx_program, usbd_rhport.offset_tx);
    pio_remove_program(usbd_rhport.pio_rx, &usb_nrzi_decoder_program, usbd_rhport.offset_rx);
    pio_remove_program(usbd_rhport.pio_rx, &usb_edge_detector_program, usbd_rhport.offset_eop);
    pio_sm_unclaim(usbd_rhport.pio_tx, usbd_rhport.sm_tx);
    pio_sm_unclaim(usbd_rhport.pio_rx, usbd_rhport.sm_rx);
    pio_sm_unclaim(usbd_rhport.pio_rx, usbd_rhport.sm_eop);
}

static void pio_usbd_connect(void) {
    if (usbd_rhport.connected) {
        return;
    }
    // usbd_rhport_t* rhport = &usbd_rhport;
    // rhport->connected = true;
    // pio_sm_set_enabled(rhport->pio_tx, rhport->sm_tx, true);
    // usbd_bus_prepare_receive(rhport);
    // rhport->pio_rx->ctrl |= (1 << rhport->sm_rx);
    // rhport->pio_rx->irq |= PIO_USB_IRQ_RX_Msk;

    // // enable the two PIO SMs that run USB RX
    // // pio_sm_set_enabled(rhport->pio_rx,  rhport->sm_rx,  true);
    // pio_sm_set_enabled(rhport->pio_rx,  rhport->sm_eop, true);

    // pio_set_irqn_source_enabled(
    //     rhport->pio_rx, 
    //     0, 
    //     (pio_interrupt_source)(pis_interrupt0 + IRQ_RX_START),
    //     true
    // );
    // irq_set_exclusive_handler(rhport->rx_irq_num, usbd_packet_handler);
    // irq_set_enabled(rhport->rx_irq_num, true);
}

static bool pio_usbd_ep_open(uint8_t dport, uint8_t epaddr, 
                             uint8_t epattr, uint16_t epsize) {
    // if ((epattr & PIO_USB_EP_ATTR_BULK) || (epattr & PIO_USB_EP_ATTR_ISOCHRONOUS)) {
    //     /* Not implemented */
    //     return false;
    // }
    if (epsize > USBD_ENDPOINT_MAX_SIZE) {
        return false;
    }
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if (ep) {
        ep->epaddr = epaddr;
        ep->epattr = epattr;
        ep->epsize = epsize;
        reset_ep_buf((epaddr & USB_EP_DIR_IN) ? &ep->buf_in : &ep->buf_out);
        return true;
    }
    return false;
}

static void pio_usbd_ep_close(uint8_t dport, uint8_t epaddr) {
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if (ep) {
        ep->epsize = 0;
        reset_ep_buf((epaddr & USB_EP_DIR_IN) ? &ep->buf_in : &ep->buf_out);
    }
}

static void pio_usbd_set_address(uint8_t dport, uint8_t daddr) {
    if (dport >= USBD_DEVICES_MAX) {
        pusb_loge("E: Set address: invalid port %d\n", dport);
        return;
    }
    usbd_device* device = &usbd_rhport.dev[dport];
    device->daddr_pending = 0;
    device->daddr = daddr;
}

static void pio_usbd_ep_xfer_abort(uint8_t dport, uint8_t epaddr) {
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if (ep == NULL) {
        return;
    }
    ep_buf_t* ep_buf = (epaddr & USB_EP_DIR_IN) 
                       ? &ep->buf_in : &ep->buf_out;
    while (ep_buf->busy) {
        tight_loop_contents();
    }
    reset_ep_buf(ep_buf);
}

static int32_t pio_usbd_ep_write(uint8_t dport, uint8_t epaddr, const uint8_t *buffer, uint16_t len) {
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if ((ep == NULL)) {
        pusb_loge("E: EP write: %02X, %s\n", 
            epaddr,
            (ep == NULL) ? "not found" : "pending xfer");
        return -1;
    }
    if ((ep->epattr & USB_EP_TYPE_ISOCHRONUS) && ep->buf_in.has_xfer) {
        pio_usbd_ep_xfer_abort(dport, epaddr);
    } else if (ep->buf_in.has_xfer) {
        // usb_loge("E: EP write: %02X, pending xfer\n", epaddr);
        return -1;
    }

    len = std::min(len, ep->epsize);

    ep->buf_in.data[0] = PIO_USB_SYNC;
    // ep->buf_in.data[1] is DATA0/1, updated in IRQ handler

    std::memcpy(ep->buf_in.data + 2, buffer, len);
    ep->buf_in.len = len + 4;

    uint16_t crc16 = calc_usb_crc16(ep->buf_in.data + 2, len);
    ep->buf_in.data[2 + len]     = crc16 & 0xff;
    ep->buf_in.data[2 + len + 1] = (crc16 >> 8) & 0xff;

    ep->buf_in.has_xfer = true;
    ep->buf_in.stalled = false;
    return len;
}

static int32_t pio_usbd_read_setup(uint8_t dport, uint8_t* buffer) {
    if (dport >= USBD_DEVICES_MAX) {
        return -1;
    }
    usbd_device* device = &usbd_rhport.dev[dport];
    if (!device->has_setup) {
        pusb_loge("E: No setup available, port: %d\n", dport);
        return -1;
    }
    uint16_t len = MIN(device->setup_len, 8);
    std::memcpy(buffer, device->setup_buf, len);
    device->has_setup = false;
    return len;
}

static int32_t pio_usbd_ep_read(uint8_t dport, uint8_t epaddr, uint8_t *buffer, uint16_t len) {
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if ((ep == NULL) || !ep->buf_out.has_xfer) {
        pusb_loge("E: EP read: %02X%s\n", 
            epaddr, (ep == NULL) ? ", not found" : "");
        return -1;
    }
    uint16_t xfer_len = std::min(len, ep->buf_out.len);
    std::memcpy(buffer, ep->buf_out.data, xfer_len);
    ep->buf_out.has_xfer = false;
    return xfer_len;
}

static void pio_usbd_ep_set_stall(uint8_t dport, uint8_t epaddr, bool stall) {
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if (ep == NULL) {
        return;
    }
    ep_buf_t* ep_buf = (epaddr & USB_EP_DIR_IN) ? &ep->buf_in : &ep->buf_out;
    ep_buf->has_xfer = false;
    ep_buf->stalled = stall;
    
    pusb_logd("D: Set stall %s, EP %02X\n", 
        stall ? "true" : "false", epaddr);
}

static bool pio_usbd_ep_stalled(uint8_t dport, uint8_t epaddr) {
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if (ep == NULL) {
        return false;
    }
    ep_buf_t* ep_buf = (epaddr & USB_EP_DIR_IN) 
                       ? &ep->buf_in : &ep->buf_out;
    return ep_buf->stalled;
}

static bool pio_usbd_ep_ready(uint8_t dport, uint8_t epaddr) {
    usbd_endpoint_t* ep = get_endpoint(dport, epaddr);
    if (ep != NULL) {
        return ((epaddr & USB_EP_DIR_IN) ?
                !ep->buf_in.has_xfer : ep->buf_out.has_xfer);
    }
    return false;
}

static uint16_t pio_usbd_get_frame(void) {
    return usbd_rhport.frame_num;
}

static bool pio_usbd_task(uint8_t dport, uint32_t* event_mask, uint32_t* ep_mask) {
    if (dport >= USBD_DEVICES_MAX) {
        return false;
    }
    if (!usbd_rhport.reset) {
        uint32_t start_time_us = time_us_32();
        bool reset = false;
        while ((usbd_bus_get_line_state(&usbd_rhport) == PIO_USB_PORT_PIN_SE0) &&
            !dma_channel_is_busy(usbd_rhport.dma_chan_tx)) {
            if ((time_us_32() - start_time_us) >= 2500) {
                reset = true;
                break;
            }
            busy_wait_us_32(1);
        }
        if (reset) {
            usbd_rhport.reset = true;
            for (uint8_t i = 0; i < USBD_DEVICES_MAX; i++) {
                usbd_device* dev = &usbd_rhport.dev[i];
                dev->daddr = 0;
                dev->daddr_pending = 0;
                dev->has_setup = false;
                for (uint8_t j = 0; j < USBD_ENDPOINTS_MAX; j++) {
                    pio_usbd_ep_close(i, j | USB_EP_DIR_OUT);
                    pio_usbd_ep_close(i, j | USB_EP_DIR_IN);
                }
                
                usbd_rhport.dev[i].store_ints(USBD_EVENT_RESET, 0);
            }
            usbd_restart_receiver(&usbd_rhport);
        }
    }
    pio_usbd_ints_t ints = usbd_rhport.dev[dport].load_clear_ints();
    *event_mask = ints.device;
    *ep_mask = ints.ep;
    return (ints.device != 0);
}

const dcd_driver_t DCD_DRIVER_PIO = {
    .init = pio_usbd_init,
    .deinit = pio_usbd_deinit, // No deinit function
    .connect = pio_usbd_connect,
    .set_address = pio_usbd_set_address,
    .ep_open = pio_usbd_ep_open,
    .ep_close = pio_usbd_ep_close,
    .ep_xfer_abort = pio_usbd_ep_xfer_abort,
    .ep_read_setup = pio_usbd_read_setup,
    .ep_read = pio_usbd_ep_read,
    .ep_write = pio_usbd_ep_write,
    .ep_set_stall = pio_usbd_ep_set_stall,
    .ep_stalled = pio_usbd_ep_stalled,
    .ep_ready = pio_usbd_ep_ready,
    .get_frame = pio_usbd_get_frame,
    .task = pio_usbd_task
};

#pragma GCC pop_options

#endif /* USBD_DEVICES_MAX */