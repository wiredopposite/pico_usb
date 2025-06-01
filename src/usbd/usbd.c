#include "usbd.h"
#if USBD_DEVICES_MAX

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <pico/unique_id.h>
#include "usbd/dcd/dcd.h"
#include "common/usb_def.h"
#include "common/usb_util.h"
#include "common/usb_log.h"

#define VERIFY_HANDLE(handle, ret) \
    do { \
        if (((handle) == NULL) || \
            ((handle)->state == USBD_STATE_DISABLED)) { \
            usb_logv("Error: !Handle %s, %s\n", __func__); \
            return ret; \
        } \
    } while (0)

#define VERIFY_COND(cond, ret) \
    do { \
        if (!(cond)) { \
            usb_logv("Error: %s, !(%s)\n", __func__, #cond); \
            return ret; \
        } \
    } while (0)

static usbd_handle_t handles[USBD_DEVICES_MAX] = {0};
static const size_t usbd_handles_size = sizeof(handles);

static void usbd_assign_unique_id(usbd_handle_t* handle) {
    pico_unique_board_id_t unique_id;
    pico_get_unique_board_id(&unique_id);
    usb_desc_string_t* str = (usb_desc_string_t*)handle->desc_serial_buf;
    str->bLength = USBD_SERIAL_BUF_SIZE;
    str->bDescriptorType = USB_DTYPE_STRING;
    for (uint8_t i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        uint8_t byte = unique_id.id[i];
        char high = (byte >> 4) & 0x0F;
        char low = byte & 0x0F;
        high = (high < 10) ? ('0' + high) : ('A' + high - 10);
        low  = (low  < 10) ? ('0' + low)  : ('A' + low  - 10);
        str->wString[i * 4 + 0] = high;
        str->wString[i * 4 + 1] = 0;
        str->wString[i * 4 + 2] = low;
        str->wString[i * 4 + 3] = 0;
    }
    char port_high = (handle->port >> 4) & 0x0F;
    char port_low  = handle->port & 0x0F;
    port_high = (port_high < 10) ? ('0' + port_high) : ('A' + port_high - 10);
    port_low  = (port_low  < 10) ? ('0' + port_low)  : ('A' + port_low  - 10);
    str->wString[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 4 + 0] = port_high;
    str->wString[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 4 + 1] = 0;
    str->wString[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 4 + 2] = port_low;
    str->wString[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 4 + 3] = 0;
}

static void usbd_stall_ctrl_ep(usbd_handle_t* handle) {
    handle->dcd_driver->ep_set_stall(handle->port, 0 | USB_EP_DIR_IN, true);
    handle->dcd_driver->ep_set_stall(handle->port, 0 | USB_EP_DIR_OUT, true);
    handle->ctrl_ep.stage = USB_CTRL_STAGE_IDLE;
    handle->ctrl_ep.complete_cb = NULL;
    usb_logd("Stall Ctrl EP Port %d\n", handle->port);
}

static void usbd_ep_complete_cb(usbd_handle_t* handle, usbd_event_t event, uint8_t epaddr) {
    handle->app_driver.ep_xfer_cb(handle, epaddr);
}

static void usbd_set_address_cb(usbd_handle_t* handle, usb_ctrl_req_t* req) {
    handle->dcd_driver->set_address(handle->port, req->wValue);
    handle->state = (req->wValue) ? USBD_STATE_ADDRESSED : USBD_STATE_DEFAULT;
    usb_logd("Set daddr: %d, port: %d\n", req->wValue, handle->port);
}

static void usbd_config_complete_cb(usbd_handle_t* handle, usb_ctrl_req_t* req) {
    handle->state = USBD_STATE_CONFIGURED;
    handle->app_driver.configured_cb(handle, handle->config_num);
}

static bool usbd_process_set_config_req(usbd_handle_t* handle, uint8_t config) {
    usb_logd("Configure Port %d, config: %d\n", handle->port, config);
    if (config == 0) {
        handle->state = USBD_STATE_ADDRESSED;
        for (uint8_t i = 1; i < USBD_ENDPOINTS_MAX; i++) {
            handle->endpoint_cb[i] = NULL;
            handle->dcd_driver->ep_close(handle->port, i | USB_EP_DIR_IN);
            handle->dcd_driver->ep_close(handle->port, i | USB_EP_DIR_OUT);
        }
        handle->config_num = 0;
        handle->app_driver.deinit_cb(handle);
        return true;
    }
    if (handle->state == USBD_STATE_CONFIGURED) {
        usb_logd("  Already configured\n");
        return true;
    }
    if (!handle->app_driver.set_config_cb(handle, config)) {
        usb_loge("  Error: Set config failed\n");
        return false;
    }
    handle->config_num = config;
    handle->ctrl_ep.complete_cb = usbd_config_complete_cb;
    return true;
}

static bool usbd_process_std_req(usbd_handle_t* handle, usb_ctrl_req_t* req) {
    switch (req->bmRequestType & USB_REQ_RECIP_Msk) {
    case USB_REQ_RECIP_DEVICE:
        switch (req->bRequest) {
        case USB_REQ_STD_SET_ADDRESS:
            /* This gets set after the ZLP response is sent */
            handle->ctrl_ep.complete_cb = usbd_set_address_cb;
            return true;
        case USB_REQ_STD_SET_CONFIG:
            return usbd_process_set_config_req(handle, req->wValue);
        case USB_REQ_STD_GET_CONFIG:
            usbd_send_ctrl_resp(handle, &handle->config_num, sizeof(uint8_t), NULL);
            return true;
        case USB_REQ_STD_SET_FEATURE:
        case USB_REQ_STD_CLEAR_FEATURE:
            /* Not supported, ignore and ZLP anyway */
            return true;
        default:
            break;
        }
        break;
    case USB_REQ_RECIP_INTERFACE:
        switch (req->bRequest) {
        case USB_REQ_STD_GET_STATUS:
            {
            /* Default itf status is zero */
            uint16_t status = 0;
            usbd_send_ctrl_resp(handle, &status, sizeof(uint16_t), NULL);
            return true;
            }
        default:
            break;
        }
        break;
    case USB_REQ_RECIP_ENDPOINT:
        if (USB_EP_NUM(req->wIndex) >= USBD_ENDPOINTS_MAX) {
            break;
        }
        switch (req->bRequest) {
        case USB_REQ_STD_SET_FEATURE:
        case USB_REQ_STD_CLEAR_FEATURE:
            /* EP feature req sets or clears STALL */
            usb_logd("Feature Req EP: %d, %s\n", req->wIndex, 
                (req->bRequest == USB_REQ_STD_SET_FEATURE) ? "SET" : "CLEAR");
            handle->dcd_driver->ep_set_stall(
                handle->port, 
                req->wIndex, 
                (req->bRequest == USB_REQ_STD_SET_FEATURE)
            );
            return true;
        case USB_REQ_STD_GET_STATUS: 
            {
            /* EP status is just stall state */
            uint8_t status[2] = {0};
            status[0] = handle->dcd_driver->ep_stalled(handle->port, req->wIndex) 
                        ? 1 : 0;
            status[1] = 0;
            usbd_send_ctrl_resp(handle, status, sizeof(status), NULL);
            return true;
            }
        default:
            break;
        }
        break;
    default:
        break;
    }
    return false;
}

static bool usbd_process_ctrl_req(usbd_handle_t* handle, usb_ctrl_req_t* req) {
    if (req->bRequest == USB_REQ_STD_GET_DESCRIPTOR) {
        return handle->app_driver.get_desc_cb(handle, req);
    }
    switch (req->bmRequestType & USB_REQ_TYPE_Msk) {
    case USB_REQ_TYPE_STANDARD:
        if (usbd_process_std_req(handle, req)) {
            return true;
        }
        /*  Fallthrough if request unhandled */
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
        return handle->app_driver.ctrl_xfer_cb(handle, req);
    default:
        return false;
    }
}

static void usbd_process_ctrl_eptx(usbd_handle_t* handle) {
    int32_t len = 0;
    usb_ctrl_req_t* req = (usb_ctrl_req_t*)handle->ctrl_ep.rx_buf;
    switch (handle->ctrl_ep.stage) {
    case USB_CTRL_STAGE_DATA_IN:
        /* dcd_ep_write is gated to configured EP size */
        len =   handle->dcd_driver->ep_write(
                    handle->port, 
                    (0 | USB_EP_DIR_IN),
                    handle->ctrl_ep.tx_buf + handle->ctrl_ep.tx_idx,
                    handle->ctrl_ep.tx_len - handle->ctrl_ep.tx_idx
                );
        if (len > 0) {
            handle->ctrl_ep.tx_idx += len;
        }
        if (handle->ctrl_ep.tx_idx < handle->ctrl_ep.tx_len) {
            /* Still more data to send, stay in DATA_IN stage */
            usb_logd("Ctrl TX: %d/%d\n", handle->ctrl_ep.tx_idx, handle->ctrl_ep.tx_len);
            break;
        }
        if ((len != handle->ctrl_ep_size) || 
            (handle->ctrl_ep.tx_len == req->wLength)) {
            /*  We can move to STATUS_OUT stage, 
                otherwise we'd stay in DATA_IN and send a ZLP next */
            handle->ctrl_ep.stage = USB_CTRL_STAGE_STATUS_OUT;
        }
        break;
    case USB_CTRL_STAGE_STATUS_IN:
        handle->ctrl_ep.stage = USB_CTRL_STAGE_IDLE;
        if (handle->ctrl_ep.complete_cb != NULL) {
            handle->ctrl_ep.complete_cb(handle, req);
            handle->ctrl_ep.complete_cb = NULL;
        }
        break;
    default:
        break;
    }
}

static void usbd_process_ctrl_eprx(usbd_handle_t* handle) {
    int32_t len = 0;
    usb_ctrl_req_t* req = (usb_ctrl_req_t*)handle->ctrl_ep.rx_buf;

    switch (handle->ctrl_ep.stage) {
    case USB_CTRL_STAGE_IDLE:
        len =   handle->dcd_driver->ep_read_setup(
                    handle->port, 
                    handle->ctrl_ep.rx_buf
                );
        if (len != sizeof(usb_ctrl_req_t)) {
            usbd_stall_ctrl_ep(handle);
            usb_logd("Error: Ctrl Setup: %d\n", len);
            return;
        }

        handle->ctrl_ep.rx_idx = len;
        handle->ctrl_ep.rx_len = len + req->wLength;

        if ((req->bmRequestType & USB_REQ_DIR_DEVTOHOST) || 
            (req->wLength == 0)) {
            /* Request complete */
            break;
        }
        if (handle->ctrl_ep.rx_len > sizeof(handle->ctrl_ep.rx_buf)) {
            usbd_stall_ctrl_ep(handle);
            usb_loge("Error: Ctrl RX buffer too small: %d/%d\n", 
                handle->ctrl_ep.rx_len, sizeof(handle->ctrl_ep.rx_buf));
            return;
        }
        handle->ctrl_ep.stage = USB_CTRL_STAGE_DATA_OUT;
        return;
    case USB_CTRL_STAGE_DATA_OUT:
        len =   handle->dcd_driver->ep_read(
                    handle->port, 
                    (0 | USB_EP_DIR_OUT), 
                    handle->ctrl_ep.rx_buf + handle->ctrl_ep.rx_idx, 
                    handle->ctrl_ep.rx_len - handle->ctrl_ep.rx_idx
                );
        if (len < 0) {
            /* Read failed */
            usbd_stall_ctrl_ep(handle);
            usb_logd("Error: Ctrl RX read failed: %d, port: %d\n", 
                len, handle->port);
            return;
        }
        handle->ctrl_ep.rx_idx += len;
        if (handle->ctrl_ep.rx_idx < handle->ctrl_ep.rx_len) {
            /* Still RX data remaining, remain in DATA_OUT stage */
            return;
        }
        break;
    case USB_CTRL_STAGE_STATUS_OUT:
        handle->dcd_driver->ep_read(
            handle->port, 
            (0 | USB_EP_DIR_OUT), 
            NULL, 
            0
        );
        handle->ctrl_ep.stage = USB_CTRL_STAGE_IDLE;
        if (handle->ctrl_ep.complete_cb != NULL) {
            handle->ctrl_ep.complete_cb(handle, req);
            handle->ctrl_ep.complete_cb = NULL;
        }
        return;
    default:
        /* Unexpected packet */
        len =   handle->dcd_driver->ep_read(
                    handle->port, 
                    (0 | USB_EP_DIR_OUT), 
                    handle->ctrl_ep.rx_buf, 
                    sizeof(handle->ctrl_ep.rx_buf)
                );
        if (len == 0) { /* ZLP */
            usb_logd("Host aborted control xfer, port: %d\n", 
                handle->port);
            handle->ctrl_ep.stage = USB_CTRL_STAGE_IDLE;
            handle->ctrl_ep.complete_cb = NULL;
            if (!handle->dcd_driver->ep_ready(handle->port, (0 | USB_EP_DIR_IN))) {
                handle->dcd_driver->ep_xfer_abort(handle->port, (0 | USB_EP_DIR_IN));
            }
        } else {
            usbd_stall_ctrl_ep(handle);
            usb_loge("Error: Ctrl RX, unexpected packet, len: %d, port: %d\n", 
                len, handle->port);
        }
        return;
    }
    /* Full request received */
    if ((req->bmRequestType & USB_REQ_DIR_Msk) == USB_REQ_DIR_DEVTOHOST) {
        handle->ctrl_ep.tx_len = 0;
        handle->ctrl_ep.tx_idx = 0;
        handle->ctrl_ep.stage = USB_CTRL_STAGE_DATA_IN;
    }
    /*  usbd_process_ctrl_req will call the app's ctrl_xfer_cb or get_desc_cb,
        if DATA_IN stage, a response should be queued using usbd_send_ctrl_resp */
    if (usbd_process_ctrl_req(handle, req)) {
        if (handle->ctrl_ep.stage == USB_CTRL_STAGE_DATA_IN) {
            if ((handle->ctrl_ep.tx_len == 0) && (req->wLength > 0)) {
                /*  app returned true but didn't queue 
                    a response, not okay! set stall */
                usbd_stall_ctrl_ep(handle);
                return;
            }
            if (handle->ctrl_ep.tx_len > req->wLength) {
                handle->ctrl_ep.tx_len = req->wLength;
            }
            usbd_process_ctrl_eptx(handle);
        } else {
            /* Send ZLP to confirm STATUS_IN stage */
            handle->dcd_driver->ep_write(
                handle->port, 
                (0 | USB_EP_DIR_IN), 
                NULL, 
                0
            );
            handle->ctrl_ep.stage = USB_CTRL_STAGE_STATUS_IN;
            usb_logd("Ctrl status in\n");
        }
    } else {
        /* Request went unhandle, set STALL */
        usbd_stall_ctrl_ep(handle);
    }
}

static void usbd_process_ctrl_ep(usbd_handle_t* handle, usbd_event_t event, uint8_t epaddr) {
    switch (event) {
    case USBD_EVENT_SETUP:
        handle->ctrl_ep.stage = USB_CTRL_STAGE_IDLE;
        handle->ctrl_ep.complete_cb = NULL;
        /* Fallthrough */
    case USBD_EVENT_EP_CMPLT:
        if (epaddr & USB_EP_DIR_IN) {
            usbd_process_ctrl_eptx(handle);
        } else {
            usbd_process_ctrl_eprx(handle);
        }
        break;
    default:
        break;
    }
}

static void usbd_process_event(usbd_handle_t* handle, usbd_event_t event, uint8_t epaddr) {
    uint8_t epnum = USB_EP_NUM(epaddr);
    switch (event) {
    case USBD_EVENT_RESET:
        usbd_reset_device(handle);
        break;
    case USBD_EVENT_SETUP:
    case USBD_EVENT_EP_CMPLT:
        if (handle->endpoint_cb[epnum] != NULL) {
            handle->endpoint_cb[epnum](handle, event, epaddr);
        } else {
            usb_logd("No endpoint callback for %02X\n", epaddr);
        }
        break;
    default:
        break;
    }
}

/* ---- Public Methods ---- */

usbd_handle_t* usbd_init(usbd_hw_type_t hw_type, const usbd_driver_t *driver, uint8_t ep0_size) {
    if ((driver == NULL) || (ep0_size == 0) || 
        (ep0_size > USBD_ENUMERATION_SIZE)) {
        return false;
    }
    if ((driver->init_cb        == NULL) || 
        (driver->deinit_cb      == NULL) || 
        (driver->set_config_cb  == NULL) ||
        (driver->configured_cb  == NULL) ||
        (driver->ctrl_xfer_cb   == NULL) ||
        (driver->ep_xfer_cb     == NULL)) {
        usb_loge("Error: Invalid driver\n");
        return NULL;
    }

    usbd_handle_t* handle = NULL;
    switch (hw_type) {
    case USBD_HW_USB:
        if (handles[USBD_DEVICES_MAX - 1].state != USBD_STATE_DISABLED) {
            usb_loge("Error: USB HW device already initialized\n");
            return NULL;
        }
        handle = &handles[USBD_DEVICES_MAX - 1];
        memset(handle, 0, sizeof(usbd_handle_t));
        handle->port = USBD_DEVICES_MAX - 1;
        handle->dcd_driver = &DCD_DRIVER_PICO;
        if (!handle->dcd_driver->init()) {
            usb_logd("Error: Failed to initialize DCD driver\n");
            return NULL;
        }
        break;
    case USBD_HW_PIO:
        for (uint8_t i = 0; i < ARRAY_SIZE(handles); i++) {
            if (!handles[i].state != USBD_STATE_DISABLED) {
                handle = &handles[i];
                memset(handle, 0, sizeof(usbd_handle_t));
                handle->port = i;
                handle->dcd_driver = &DCD_DRIVER_PIO;
                break;
            }
        }
        if (handle == NULL) {
            usb_loge("Error: No handles available\n");
            return NULL;
        }
        if ((handle->port == 0) && !handle->dcd_driver->init()) {
            /* Only init port 0 for PIO */
            usb_logd("Error: Failed to initialize DCD driver\n");
            return NULL;
        }
        break;
    default:
        usb_loge("Error: Invalid HW type\n");
        return NULL;
    }

    handle->hw_type = hw_type;
    handle->app_driver = *driver;
    handle->ctrl_ep_size = ep0_size;
    handle->endpoint_cb[0] = usbd_process_ctrl_ep;
    usbd_assign_unique_id(handle);
    handle->state = USBD_STATE_DEFAULT;

    usb_logi("USBD Init: %s, port: %d\n", 
        (handle->hw_type == USBD_HW_USB) ? "USB" : "PIO", handle->port);
    return handle;
}

void usbd_deinit(usbd_hw_type_t hw_type) {
    VERIFY_COND((uint8_t)hw_type <= USBD_HW_PIO, );
    for (uint8_t i = 0; i < USBD_DEVICES_MAX; i++) {
        usbd_handle_t* handle = &handles[i];
        if ((handle->state == USBD_STATE_DISABLED) || 
            (handle->hw_type != hw_type)) {
            continue;
        }
        usb_logi("Deinit USBD Port %d\n", handle->port);
        handle->app_driver.deinit_cb(handle);
        handle->dcd_driver->deinit();
        memset(handle, 0, sizeof(usbd_handle_t));
    }
}

void usbd_task(void) {
    for (uint8_t i = 0; i < ARRAY_SIZE(handles); i++) {
        usbd_handle_t* handle = &handles[i];
        if (handle->state == USBD_STATE_DISABLED) {
            continue;
        }
        uint32_t events = 0;
        uint32_t eps = 0;
        if (!handle->dcd_driver->task(handle->port, &events, &eps)) {
            continue;
        }
        if (events & USBD_EVENT_RESET) {
            events &= ~USBD_EVENT_RESET;
            usb_logv("EVT: Reset\n");
            usbd_process_event(handle, USBD_EVENT_RESET, 0);
        }
        if (events & USBD_EVENT_SETUP) {
            events &= ~USBD_EVENT_SETUP;
            usb_logv("EVT: Setup\n");
            usbd_process_event(handle, USBD_EVENT_SETUP, 0 | USB_EP_DIR_OUT);
        }
        if ((events & USBD_EVENT_EP_CMPLT) && (eps != 0)) {
            events &= ~USBD_EVENT_EP_CMPLT;
            usb_logv("EVT: EP cmplt\n");
            for (uint8_t j = 0; (j < (USBD_ENDPOINTS_MAX * 2)) && (eps != 0); j++) {
                if (eps & (1 << j)) {
                    eps &= ~(1 << j);
                    uint8_t epaddr = USB_EPADDR_FROM_IDX(j);
                    usb_logv("  EP %s: %02X\n", (epaddr & USB_EP_DIR_IN) ? "IN" : "OUT", epaddr);
                    usbd_process_event(handle, USBD_EVENT_EP_CMPLT, epaddr);
                }
            }
        }
        if (events & USBD_EVENT_SUSPEND) {
            events &= ~USBD_EVENT_SUSPEND;
            usb_logv("EVT: Suspend\n");
        }
        if (events & USBD_EVENT_WAKEUP) {
            events &= ~USBD_EVENT_WAKEUP;
            usb_logv("EVT: Wakeup\n");
        }
    }
}

void usbd_connect(usbd_hw_type_t hw_type) {
    VERIFY_COND((uint8_t)hw_type <= USBD_HW_PIO, );
    if (hw_type == USBD_HW_USB) {
        DCD_DRIVER_PICO.connect();
    } else if (hw_type == USBD_HW_PIO) {
        DCD_DRIVER_PIO.connect();
    }
}

void usbd_reset_device(usbd_handle_t* handle) {
    VERIFY_HANDLE(handle, );
    handle->state = USBD_STATE_DEFAULT;
    handle->app_driver.deinit_cb(handle);
    memset(&handle->ctrl_ep, 0, sizeof(handle->ctrl_ep));
    for (uint8_t i = 0; i < USBD_ENDPOINTS_MAX; i++) {
        handle->endpoint_cb[i] = NULL;
        handle->dcd_driver->ep_close(handle->port, i | USB_EP_DIR_IN);
        handle->dcd_driver->ep_close(handle->port, i | USB_EP_DIR_OUT);
    }
    handle->dcd_driver->set_address(handle->port, 0);
    handle->dcd_driver->ep_open(handle->port, 0 | USB_EP_DIR_IN,  
                                USB_EP_TYPE_CONTROL, handle->ctrl_ep_size);
    handle->dcd_driver->ep_open(handle->port, 0 | USB_EP_DIR_OUT, 
                                USB_EP_TYPE_CONTROL, handle->ctrl_ep_size);
    handle->endpoint_cb[0] = usbd_process_ctrl_ep;
    handle->app_driver.init_cb(handle);
    usb_logd("Reset Device Port %d\n", handle->port);
}

bool usbd_send_ctrl_resp(usbd_handle_t* handle, const void* buffer, 
                         uint16_t len, usbd_request_cb complete_cb) {
    VERIFY_HANDLE(handle, false);
    if (handle->ctrl_ep.stage == USB_CTRL_STAGE_DATA_IN) {
        uint16_t length = MIN(len, sizeof(handle->ctrl_ep.tx_buf));
        memcpy(handle->ctrl_ep.tx_buf, buffer, length);
        handle->ctrl_ep.tx_idx = 0;
        handle->ctrl_ep.tx_len = length;
        handle->ctrl_ep.complete_cb = complete_cb;
        return true;
    }
    return false;
}

bool usbd_ep_ready(usbd_handle_t* handle, uint8_t epaddr) {
    VERIFY_HANDLE(handle, false);
    VERIFY_COND(USB_EP_NUM(epaddr) < USBD_ENDPOINTS_MAX, false);
    VERIFY_COND(handle->state == USBD_STATE_CONFIGURED, false);
    return handle->dcd_driver->ep_ready(handle->port, epaddr);
}

void usbd_ep_flush(usbd_handle_t* handle, uint8_t epaddr) {
    VERIFY_HANDLE(handle, );
    VERIFY_COND(USB_EP_NUM(epaddr) < USBD_ENDPOINTS_MAX, );
    VERIFY_COND(handle->state == USBD_STATE_CONFIGURED, );
    handle->dcd_driver->ep_xfer_abort(handle->port, epaddr);
}

int32_t usbd_ep_write(usbd_handle_t* handle, uint8_t epaddr, const void* buffer, uint16_t len) {
    VERIFY_HANDLE(handle, -1);
    VERIFY_COND(USB_EP_NUM(epaddr) < USBD_ENDPOINTS_MAX, -1);
    VERIFY_COND(handle->state == USBD_STATE_CONFIGURED, -1);
    return handle->dcd_driver->ep_write(handle->port, epaddr, buffer, len);
}

int32_t usbd_ep_read(usbd_handle_t* handle, uint8_t epaddr, void* buffer, uint16_t len) {
    VERIFY_HANDLE(handle, -1);
    VERIFY_COND(USB_EP_NUM(epaddr) < USBD_ENDPOINTS_MAX, -1);
    VERIFY_COND(handle->state == USBD_STATE_CONFIGURED, -1);
    return handle->dcd_driver->ep_read(handle->port, epaddr, buffer, len);
}

bool usbd_ep_open(usbd_handle_t* handle, uint8_t epaddr, uint8_t epattr, uint16_t epsize) {
    VERIFY_HANDLE(handle, false);
    VERIFY_COND(handle->state == USBD_STATE_ADDRESSED, false);
    uint8_t epnum = USB_EP_NUM(epaddr);
    /* App facing ep open shouldn't touch EP 0 */
    if ((epnum > 0) && (epnum < USBD_ENDPOINTS_MAX)) {
        handle->endpoint_cb[epnum] = usbd_ep_complete_cb;
        return handle->dcd_driver->ep_open(handle->port, epaddr, epattr, epsize);
    }
    return false;
}

bool usbd_configure_all_eps(usbd_handle_t* handle, const void* desc_config) {
    VERIFY_HANDLE(handle, false);
    VERIFY_COND(desc_config != NULL, false);
    VERIFY_COND(handle->state == USBD_STATE_ADDRESSED, false);

    if (((usb_desc_config_t*)desc_config)->bDescriptorType != USB_DTYPE_CONFIGURATION) {
        usb_loge("Error: Invalid descriptor type\n");
        return false;
    }

    const uint8_t* desc_p = (const uint8_t*)desc_config;
    const uint8_t* end_p = 
        desc_p + ((usb_desc_config_t*)desc_config)->wTotalLength;
    uint8_t num_eps = 0;

    while (desc_p < end_p) {
        const usb_desc_header_t* desc_hdr = (const usb_desc_header_t*)desc_p;
        if (desc_hdr->bLength == 0) {
            break;
        }
        if (desc_hdr->bDescriptorType == USB_DTYPE_ENDPOINT) {
            const usb_desc_endpoint_t* desc_ep = 
                (const usb_desc_endpoint_t*)desc_p;
            uint8_t epnum = USB_EP_NUM(desc_ep->bEndpointAddress);

            if (epnum < USBD_ENDPOINTS_MAX) {
                bool open = usbd_ep_open(
                                handle, 
                                desc_ep->bEndpointAddress, 
                                desc_ep->bmAttributes, 
                                desc_ep->wMaxPacketSize
                            );
                if (open) {
                    handle->endpoint_cb[epnum] = usbd_ep_complete_cb;
                    num_eps++;
                } else {
                    usb_loge("  Failed to open EP %02X\n", 
                        desc_ep->bEndpointAddress);
                    return false;
                }
            } else {
                usb_loge("  EP %02X out of range\n",
                    desc_ep->bEndpointAddress);
                return false;
            }
        }
        desc_p = desc_p + desc_hdr->bLength;
    }
    return (num_eps > 0);
}

const usb_desc_string_t* usbd_get_desc_string_serial(usbd_handle_t* handle) {
    VERIFY_HANDLE(handle, NULL);
    return (const usb_desc_string_t*)handle->desc_serial_buf;
}

#endif /* USBD_DEVICES_MAX */