#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <stdio.h>
#include "class/hid_def.h"
#include "hid_desc.h"
#include "hid.h"

#define BTN_INTERVAL_US         (1000U*1000U)

static uint32_t btn_start_us = 0;
static hid_report_in_t report_in = {0};
static uint8_t idle_rate = 0;
static uint8_t protocol = 1;

static void reset_state(void) {
    idle_rate = 0;
    protocol = 1;
    report_in.buttons = 0;
    report_in.dpad = HID_DPAD_CENTER;
    report_in.joystick_lx = HID_JOYSTICK_MID;
    report_in.joystick_ly = HID_JOYSTICK_MID;
    report_in.joystick_rx = HID_JOYSTICK_MID;
    report_in.joystick_ry = HID_JOYSTICK_MID;
    report_in.vendor = 0;
}

static void hid_init_cb(usbd_handle_t* handle) {
    (void)handle;
    printf("HID: Init\n");
    reset_state();
}

static void hid_deinit_cb(usbd_handle_t* handle) {
    (void)handle;
    printf("HID: Deinit\n");
}

static bool hid_set_config_cb(usbd_handle_t* handle, uint8_t config) {
    printf("HID: Set config: %d\n", config);
    return usbd_configure_all_eps(handle, &HID_DESC_CONFIG);
}

static void hid_configured_cb(usbd_handle_t* handle, uint8_t config) {
    printf("HID: Mounted, Config: %d\n", config);
}

static bool hid_get_desc_cb(usbd_handle_t* handle, const usb_ctrl_req_t* req) {
    switch (USB_DESC_TYPE(req->wValue)) {
    case USB_DTYPE_DEVICE:
        printf("HID: Get desc device\n");
        return  usbd_send_ctrl_resp(
                    handle, 
                    &HID_DESC_DEVICE, 
                    HID_DESC_DEVICE.bLength, 
                    NULL
                );
    case USB_DTYPE_CONFIGURATION:
        printf("HID: Get desc config\n");
        return  usbd_send_ctrl_resp(
                    handle, 
                    &HID_DESC_CONFIG, 
                    sizeof(HID_DESC_CONFIG), 
                    NULL
                );
    case USB_DTYPE_STRING:
        {
        const uint8_t idx = req->wValue & 0xFF;
        printf("HID: Get desc string: %d\n", idx);
        if (idx < ARRAY_SIZE(HID_DESC_STR)) {
            return  usbd_send_ctrl_resp(
                        handle, 
                        HID_DESC_STR[idx], 
                        HID_DESC_STR[idx]->bLength, 
                        NULL
                    );
        } else if (idx == HID_DESC_DEVICE.iSerialNumber) {
            const usb_desc_string_t* serial = 
                usbd_get_desc_string_serial(handle);
            return  usbd_send_ctrl_resp(
                        handle, 
                        serial, 
                        serial->bLength, 
                        NULL
                    );
        }
        }
    case USB_DTYPE_HID_REPORT:
        printf("HID: Get desc HID report\n");
        return  usbd_send_ctrl_resp(
                    handle, 
                    HID_DESC_REPORT, 
                    sizeof(HID_DESC_REPORT), 
                    NULL
                );
    case USB_DTYPE_HID:
        {
        printf("HID: Get desc HID class\n");
        return  usbd_send_ctrl_resp(
                    handle, 
                    &HID_DESC_CONFIG, 
                    HID_DESC_CONFIG.hid_desc.bLength, 
                    NULL
                );
        }
    default:
        return false;
    }
}

static bool hid_ctrl_xfer_cb(usbd_handle_t* handle, const usb_ctrl_req_t* req) {
    if ((req->bmRequestType & (USB_REQ_TYPE_Msk | USB_REQ_RECIP_Msk)) !=
        (USB_REQ_TYPE_CLASS | USB_REQ_RECIP_INTERFACE)) {
        return false;
    }
    switch (req->bRequest) {
    case USB_REQ_HID_SET_REPORT:
        printf("HID: Set report\n");
        return true;
    case USB_REQ_HID_GET_REPORT:
        printf("HID: Get report: %d\n", req->wValue);
        return usbd_send_ctrl_resp(handle, &report_in, sizeof(report_in), NULL);
    case USB_REQ_HID_SET_IDLE:
        printf("HID: Set idle: %d\n", req->wValue);
        idle_rate = (req->wValue >> 8) & 0xFF;
        return true;
    case USB_REQ_HID_GET_IDLE:
        printf("HID: Get idle: %d\n", req->wValue);
        return usbd_send_ctrl_resp(handle, &idle_rate, sizeof(idle_rate), NULL);
    case USB_REQ_HID_SET_PROTOCOL:
        printf("HID: Set protocol: %d\n", req->wValue);
        if (req->wValue <= 1) {
            protocol = req->wValue;
            return true;
        }
        break;
    case USB_REQ_HID_GET_PROTOCOL:
        printf("HID: Get protocol: %d\n", req->wValue);
        return usbd_send_ctrl_resp(handle, &protocol, sizeof(protocol), NULL);
    default:
        printf("HID: Unknown req: %02X\n", req->bRequest);
        return false;
    }
}

static void hid_ep_xfer_cb(usbd_handle_t* handle, uint8_t epaddr) {
    (void)handle;
    if (USB_EP_DIR(epaddr) == USB_EP_DIR_IN) {
        /* Report sent */
    }
}

usbd_handle_t* hid_init(usbd_hw_type_t hw_type) {
    btn_start_us = time_us_32();

    /* USB device driver, does not need to be static */
    usbd_driver_t driver = {
        .init_cb        = hid_init_cb,
        .deinit_cb      = hid_deinit_cb,
        .get_desc_cb    = hid_get_desc_cb,
        .set_config_cb  = hid_set_config_cb,
        .configured_cb  = hid_configured_cb,
        .ctrl_xfer_cb   = hid_ctrl_xfer_cb,
        .ep_xfer_cb     = hid_ep_xfer_cb,
    };
    return usbd_init(hw_type, &driver, HID_CTRL_EP_SIZE);
}

void hid_task(usbd_handle_t* handle) {
    if (((time_us_32() - btn_start_us) >= BTN_INTERVAL_US) &&
        usbd_ep_ready(handle, HID_EPADDR_IN)) {
        /* Toggle buttons */
        report_in.buttons ^= HID_BUTTONS_L3 | HID_BUTTONS_A;
        btn_start_us = time_us_32();
        usbd_ep_write(handle, HID_EPADDR_IN, &report_in, sizeof(report_in));
    }
}