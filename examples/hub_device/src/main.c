#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <stdio.h>

#include "usbd.h"
#include "usb_def.h"
#include "usb_util.h"
#include "class/hub_def.h"
#include "hid.h"
#include "cdc.h"

#define HUB_CTRL_EP_SIZE    64U
#define HUB_EPADDR_IN       (1U | USB_EP_DIR_IN)

typedef enum {
    PORT_HUB = 0,
    PORT_HID,
    PORT_CDC,
    PORT_COUNT
} device_port_t;
_Static_assert(PORT_COUNT <= USBD_DEVICES_MAX, "PORT_COUNT must be <= USBD_DEVICES_MAX");

/* Descriptors */

typedef struct __attribute__((packed)) {
    usb_desc_config_t   config;
    usb_desc_itf_t      hub_itf;
    usb_desc_endpoint_t hub_ep;
} usb_desc_config_hub_t;

static const usb_desc_device_t HUB_DESC_DEVICE = {
    .bLength            = sizeof(usb_desc_device_t),
    .bDescriptorType    = USB_DTYPE_DEVICE,
    .bcdUSB             = USB_BCD_VERSION_1_1,
    .bDeviceClass       = USB_CLASS_HUB,
    .bDeviceSubClass    = USB_SUBCLASS_HUB,
    .bDeviceProtocol    = USB_PROTOCOL_HUB_FULL_SPEED,
    .bMaxPacketSize0    = HUB_CTRL_EP_SIZE,
    .idVendor           = 0x1A40,
    .idProduct          = 0x0101,
    .bcdDevice          = 0x0111,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

enum {
    ITF_NUM_HUB = 0,
    ITF_NUM_TOTAL
};

static const usb_desc_config_hub_t HUB_DESC_CONFIG = {
    .config = {
        .bLength                = sizeof(usb_desc_config_t),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(usb_desc_config_hub_t),
        .bNumInterfaces         = ITF_NUM_TOTAL,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = USB_ATTR_RESERVED    | 
                                  USB_ATTR_BUS_POWERED |
                                  USB_ATTR_REMOTE_WAKEUP,
        .bMaxPower              = 50, // 100mA
    },
    .hub_itf = {
        .bLength                = sizeof(usb_desc_itf_t),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = ITF_NUM_HUB,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_HUB,
        .bInterfaceSubClass     = 0x00,
        .bInterfaceProtocol     = 0x00,
        .iInterface             = 0
    },
    .hub_ep = {
        .bLength                = sizeof(usb_desc_endpoint_t),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = HUB_EPADDR_IN,
        .bmAttributes           = USB_EP_TYPE_INTERRUPT,
        .wMaxPacketSize         = 1, // Downstreams status change mask, 1 byte for <= 7 ports
        .bInterval              = 12
    }
};

static const usb_desc_string_t HUB_DESC_STR_LANGUAGE     = USB_ARRAY_DESC(0x0409);
static const usb_desc_string_t HUB_DESC_STR_MANUFACTURER = USB_STRING_DESC("WiredOpposite");
static const usb_desc_string_t HUB_DESC_STR_PRODUCT      = USB_STRING_DESC("PIO USB Hub");
static const usb_desc_string_t* HUB_DESC_STR[] = {
    &HUB_DESC_STR_LANGUAGE,
    &HUB_DESC_STR_MANUFACTURER,
    &HUB_DESC_STR_PRODUCT
};

static const usb_desc_hub_t HUB_DESC_HUB = {
    .bLength                = 8,
    .bDescriptorType        = USB_DTYPE_HUB,
    .bNumberOfPorts         = PORT_COUNT - 1,
    .wHubCharacteristics    = USB_HUB_CHAR_POWER_SWITCH_INDIVIDUAL  |
                              USB_HUB_CHAR_COMPOUND_DEVICE          |
                              USB_HUB_CHAR_THINKTIME_8_FS_BITS,
    .bPowerOnToPowerGood    = 0x16,
    .bHubControlCurrent     = 0xFA,
    .bRemoveAndPowerMask    = { (1U << PORT_HUB) | /* Mask indicates non-removable ports */
                                (1U << PORT_HID) |
                                (1U << PORT_CDC) }
};

/* State tracking */

typedef struct {
    usbd_handle_t*  handle;
    uint32_t        status;
} device_state_t;

static device_state_t devices[PORT_COUNT] = {0};
static uint8_t        ds_status = 0; /* Downstream ports with unreported status changes */

/* Helpers */

static bool hub_feature_req(usbd_handle_t* handle, const usb_ctrl_req_t* req) {
    const uint8_t feature = req->wValue & 0xFF;
    const uint8_t ds_port = req->wIndex & 0xFF;
    const uint8_t prev_ds_status = ds_status;

    if ((feature >= 32) || (ds_port >= PORT_COUNT)) {
        return false;
    }
    
    switch (req->bRequest) {
    case USB_REQ_STD_SET_FEATURE:
        printf("HUB: Set feature %d, port: %d\n", feature, ds_port);
        if (devices[ds_port].status & (1U << feature)) {
            break; /* No change */
        }
        devices[ds_port].status |= (1U << feature);
        if (feature < USB_FEATURE_C_PORT_Pos) {
            /* Set port bit in status change mask */
            ds_status |= (1U << ds_port);
        }
        if (feature == USB_FEATURE_PORT_RESET) {
            usbd_reset_device(devices[ds_port].handle); /* Reset downstream device */
            devices[ds_port].status &= ~(1U << USB_FEATURE_PORT_RESET);    /* Clear PORT reset*/
            devices[ds_port].status |=  (1U << USB_FEATURE_C_PORT_RESET) | /* Set C_PORT reset, will be cleared by host */
                                        (1U << USB_FEATURE_PORT_ENABLE);   /* Set PORT enable */    
        }
        break;
    case USB_REQ_STD_CLEAR_FEATURE:
        printf("HUB: Clear feature %d, port: %d\n", feature, ds_port);
        if (!(devices[ds_port].status & (1U << feature))) {
            break; /* No change */
        }
        devices[ds_port].status &= ~(1U << feature);
        if (feature < USB_FEATURE_C_PORT_Pos) {
            /* Set port bit in status change mask */
            ds_status |= (1U << ds_port);
        }
        break;
    default:
        return false;
    }
    if (ds_status != prev_ds_status) {
        /*  Report downstream status change to host. 
            If we've already written the EP and the host 
            hasn't read it yet, flush the stale data  */
        usbd_ep_flush(handle, HUB_EPADDR_IN);
        usbd_ep_write(
            handle, 
            HUB_EPADDR_IN, 
            &ds_status, 
            sizeof(ds_status)
        );
    }
    return true;
}

/* Device driver */

static void hub_init_cb(usbd_handle_t* handle) {
    (void)handle;
    printf("HUB: Init\n");
    /* Reset feature mask */
    for (uint8_t port = 1; port < PORT_COUNT; port++) {
        devices[port].status = (1U << USB_FEATURE_PORT_CONNECTION)  |
                               (1U << USB_FEATURE_C_PORT_CONNECTION);
    }
}

static void hub_deinit_cb(usbd_handle_t* handle) {
    (void)handle;
    printf("HUB: Deinit\n");
}

static bool hub_get_desc_cb(usbd_handle_t* handle, const usb_ctrl_req_t* req) {
    switch (USB_DESC_TYPE(req->wValue)) {
    case USB_DTYPE_DEVICE:
        printf("HUB: Get desc device\n");
        return  usbd_send_ctrl_resp(
                    handle, 
                    &HUB_DESC_DEVICE, 
                    sizeof(HUB_DESC_DEVICE), 
                    NULL
                );
    case USB_DTYPE_CONFIGURATION:
        printf("HUB: Get desc config\n");
        return  usbd_send_ctrl_resp(
                    handle, 
                    &HUB_DESC_CONFIG, 
                    sizeof(HUB_DESC_CONFIG), 
                    NULL
                );
    case USB_DTYPE_STRING:
        {
        const uint8_t idx = req->wValue & 0xFF;
        printf("HUB: Get desc string, idx: %d\n", idx);
        if (idx < ARRAY_SIZE(HUB_DESC_STR)) {
            return  usbd_send_ctrl_resp(
                        handle, 
                        HUB_DESC_STR[idx], 
                        HUB_DESC_STR[idx]->bLength, 
                        NULL
                    );
        } else if (idx == HUB_DESC_DEVICE.iSerialNumber) {
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
        break;
    case USB_DTYPE_HUB:
        printf("HUB: Get desc hub\n");
        return  usbd_send_ctrl_resp(
                    handle, 
                    &HUB_DESC_HUB, 
                    HUB_DESC_HUB.bLength, 
                    NULL
                );
    default:
        return false;
    }
}

static bool hub_set_config_cb(usbd_handle_t* handle, uint8_t config) {
    (void)handle;
    (void)config;
    printf("HUB: Set config req: %d\n", config);
    return usbd_configure_all_eps(handle, &HUB_DESC_CONFIG);
}

static void hub_configured_cb(usbd_handle_t* handle, uint8_t config) {
    (void)handle;
    printf("HUB: Configured, #%d\n", config);
}

static bool hub_ctrl_xfer(usbd_handle_t* handle, const usb_ctrl_req_t* req) {
    switch (req->bmRequestType & USB_REQ_TYPE_Msk) {
    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {
        case USB_REQ_STD_GET_STATUS:
            {
            uint16_t status = 0; // Bus powered (0) | remote wakeup disabled (0)
            return usbd_send_ctrl_resp(handle, &status, sizeof(status), NULL);
            }
        default:
            break;
        }
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest) {
        case USB_REQ_STD_SET_FEATURE:
        case USB_REQ_STD_CLEAR_FEATURE:
            return hub_feature_req(handle, req);
        case USB_REQ_STD_GET_STATUS:
            if (req->wIndex >= PORT_COUNT) {
                break;
            }
            return  usbd_send_ctrl_resp(
                        handle, 
                        &devices[req->wIndex].status, 
                        sizeof(devices[req->wIndex].status),
                        NULL
                    );
        default:
            break;
        }
        break;
    default:
        break;
    }
    return false;
}

static void hub_ep_xfer_cb(usbd_handle_t* handle, uint8_t epaddr) {
    /* EP IN complete, clear downstream status mask */
    ds_status = 0;
}

/* Main */

int main(void) {
    /* Clock must be 240MHz to use PIO USB */
    set_sys_clock_khz(240000, true);
    stdio_init_all();

    usbd_driver_t hub_driver = {
        .init_cb        = hub_init_cb,
        .deinit_cb      = hub_deinit_cb,
        .get_desc_cb    = hub_get_desc_cb,
        .set_config_cb  = hub_set_config_cb,
        .configured_cb  = hub_configured_cb,
        .ctrl_xfer_cb   = hub_ctrl_xfer,
        .ep_xfer_cb     = hub_ep_xfer_cb,
    };

    devices[PORT_HUB].handle = usbd_init(USBD_HW_PIO, &hub_driver, HUB_CTRL_EP_SIZE);
    if (devices[PORT_HUB].handle == NULL) {
        panic("Failed to initialize HUB, port: %d\n", PORT_HUB);
        return -1;
    }

    devices[PORT_HID].handle = hid_init(USBD_HW_PIO);
    if (devices[PORT_HID].handle == NULL) {
        panic("Failed to initialize HID, port: %d\n", PORT_HID);
        return -1;
    }

    devices[PORT_CDC].handle = cdc_init(USBD_HW_PIO);
    if (devices[PORT_CDC].handle == NULL) {
        panic("Failed to initialize CDC, port: %d\n", PORT_CDC);
        return -1;
    }

    usbd_set_connected(devices[PORT_HUB].handle, true);

    while (true) {
        usbd_task();
        hid_task(devices[PORT_HID].handle);
        cdc_task(devices[PORT_CDC].handle);
        sleep_ms(1);
    }
}