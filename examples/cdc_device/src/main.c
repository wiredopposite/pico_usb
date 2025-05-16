#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usbd.h"
#include "cdc_desc.h"

#define CDC_BAUDRATE    115200U
#define CDC_WORD_SIZE   8U

typedef struct {
    usb_cdc_line_coding_t line_coding;
    uint16_t line_state;
    bool dtr;
    bool rts;
    uint8_t rx_buf[CDC_DATA_EP_SIZE_OUT];
} cdc_itf_t;

static cdc_itf_t cdc_itf = {0};

static void cdc_init_cb(usbd_handle_t* handle) {
    (void)handle;

    cdc_itf.line_coding.dwDTERate = CDC_BAUDRATE;
    cdc_itf.line_coding.bCharFormat = USB_CDC_1_STOP_BITS;
    cdc_itf.line_coding.bParityType = USB_CDC_NO_PARITY;
    cdc_itf.line_coding.bDataBits = CDC_WORD_SIZE;

    cdc_itf.line_state = 0U;
    cdc_itf.dtr = false;
    cdc_itf.rts = false;
}

static void cdc_deinit_cb(usbd_handle_t* handle) {
    (void)handle;
}

static bool cdc_get_desc_cb(usbd_handle_t* handle, const usbd_ctrl_req_t* req) {
    switch (USB_DESC_TYPE(req->wValue)) {
    case USB_DTYPE_DEVICE:
        return usbd_send_ctrl_resp(handle, &CDC_DESC_DEVICE, 
                                   sizeof(CDC_DESC_DEVICE), NULL);
    case USB_DTYPE_CONFIGURATION:
        return usbd_send_ctrl_resp(handle, &CDC_DESC_CONFIG, 
                                   sizeof(CDC_DESC_CONFIG), NULL);
    case USB_DTYPE_STRING:
        {
        const uint8_t idx = req->wValue & 0xFF;
        if (idx < ARRAY_SIZE(CDC_DESC_STR)) {
            return usbd_send_ctrl_resp(handle, CDC_DESC_STR[idx], 
                                       CDC_DESC_STR[idx]->bLength, NULL);
        } else if (idx == CDC_DESC_DEVICE.iSerialNumber) {
            const usb_desc_string_t* serial = usbd_get_desc_string_serial(handle);
            return usbd_send_ctrl_resp(handle, serial, serial->bLength, NULL);
        }
        }
        break;
    default:
        break;
    }
    return false;
}

static bool cdc_ctrl_xfer_cb(usbd_handle_t* handle, const usbd_ctrl_req_t* req) {
    if ((req->bmRequestType & (USB_REQ_TYPE_Msk | USB_REQ_RECIP_Msk)) !=
        (USB_REQ_TYPE_CLASS | USB_REQ_RECIP_INTERFACE)) {
        printf("CDC: Invalid request type: %02X\n", req->bmRequestType);
        return false;
    }
    switch (req->bRequest) {
    case USB_REQ_CDC_GET_LINE_CODING:
        printf("CDC: Get line coding\n");
        return usbd_send_ctrl_resp(handle, &cdc_itf.line_coding, 
                                   sizeof(cdc_itf.line_coding), NULL);
    case USB_REQ_CDC_SET_LINE_CODING:
        printf("CDC: Set line coding\n");
        if (req->wLength != sizeof(cdc_itf.line_coding)) {
            printf("CDC: Invalid line coding length: %d\n", req->wLength);
            return false;
        }
        memcpy(&cdc_itf.line_coding, req->data, sizeof(cdc_itf.line_coding));
        printf("CDC: Baudrate: %d, Char format: %d, Parity: %d, Data bits: %d\n",
               cdc_itf.line_coding.dwDTERate,
               cdc_itf.line_coding.bCharFormat,
               cdc_itf.line_coding.bParityType,
               cdc_itf.line_coding.bDataBits);
        return true;
    case USB_REQ_CDC_SET_CONTROL_LINE_STATE:
        cdc_itf.line_state = req->wValue;
        cdc_itf.dtr = ((cdc_itf.line_state & USB_CDC_CONTROL_LINE_DTR) != 0);
        cdc_itf.rts = ((cdc_itf.line_state & USB_CDC_CONTROL_LINE_RTS) != 0);
        printf("CDC: Set control line state: DTR=%d, RTS=%d\n", 
            cdc_itf.dtr, cdc_itf.rts);
        return true;
    case USB_REQ_CDC_SEND_BREAK:
        printf("CDC: Send break\n");
        return true;
    default:
        printf("CDC: Unknown req: %02X\n", req->bRequest);
        break;
    }
    return false;
}

static bool cdc_set_config_cb(usbd_handle_t* handle, uint8_t config) {
    (void)config;
    return usbd_configure_all_eps(handle, &CDC_DESC_CONFIG);
}

static void cdc_configured_cb(usbd_handle_t* handle, uint8_t config) {
    (void)handle;
    (void)config;
}

static void cdc_ep_xfer_cb(usbd_handle_t* handle, uint8_t epaddr) {
    (void)handle;
    switch (epaddr) {
    case CDC_DATA_EPADDR_IN:
        /* Data sent */
        break;
    case CDC_DATA_EPADDR_OUT:
        /* Data received */
        {
        int32_t len = usbd_ep_read(handle, CDC_DATA_EPADDR_OUT, 
                                   cdc_itf.rx_buf, sizeof(cdc_itf.rx_buf));
        if (len >= 0) {
            printf("CDC: Received %d bytes%s", len, (len > 0) ? ":\n" : "");
            for (uint16_t i = 0; i < len; i++) {
                if (i && ((i % 8) == 0)) {
                    printf("\n");
                }
                printf(" %02X", cdc_itf.rx_buf[i]);
            }
            printf("\n");
        } else {
            printf("CDC: EP read error: %d\n", len);
        }
        }
        break;
    default:
        break;
    }
}

int main(void) {
    stdio_init_all();

    usbd_driver_t cdc_driver = {
        .init_cb = cdc_init_cb,
        .deinit_cb = cdc_deinit_cb,
        .get_desc_cb = cdc_get_desc_cb,
        .set_config_cb = cdc_set_config_cb,
        .configured_cb = cdc_configured_cb,
        .ctrl_xfer_cb = cdc_ctrl_xfer_cb,
        .ep_xfer_cb = cdc_ep_xfer_cb
    };

    usbd_handle_t* cdc_handle = usbd_init(USBD_HW_USB, &cdc_driver, CDC_CTRL_EP_SIZE);
    if (cdc_handle == NULL) {
        printf("CDC: Failed to init USB device\n");
        return -1;
    }

    usbd_set_connected(cdc_handle, true);
    printf("CDC: USB device connected\n");

    uint32_t start = time_us_32();
    const uint32_t interval = 2000000; // 2 seconds
    const char msg[] = "Hello from CDC!\n";

    while (true) {
        usbd_task();

        if (((time_us_32() - start) > interval) && 
            usbd_ep_ready(cdc_handle, CDC_DATA_EPADDR_IN)) {
            usbd_ep_write(cdc_handle, CDC_DATA_EPADDR_IN, msg, sizeof(msg));
            start = time_us_32();
        }

        sleep_ms(1);
    }
}