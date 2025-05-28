#pragma once

#ifndef USBD_DEVICES_MAX
#define USBD_DEVICES_MAX        1
#endif

#if USBD_DEVICES_MAX

#include <stdint.h>
#include <stdbool.h>
#include <pico/unique_id.h>
#include "common/usb_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef USBD_ENDPOINTS_MAX
#define USBD_ENDPOINTS_MAX      16
#endif

#ifndef USBD_ENDPOINT_MAX_SIZE
#define USBD_ENDPOINT_MAX_SIZE  64
#endif

#ifndef USBD_ENUMERATION_SIZE
#define USBD_ENUMERATION_SIZE   256
#endif

#define USBD_SERIAL_BUF_SIZE (sizeof(usb_desc_string_t) + (PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 4) + 4)

typedef enum {
    USBD_STATE_DISABLED = 0,
    USBD_STATE_DEFAULT,
    USBD_STATE_ADDRESSED,
    USBD_STATE_CONFIGURED,
} usbd_state_t;

typedef enum {
    USB_CTRL_STAGE_IDLE = 0,
    USB_CTRL_STAGE_DATA_OUT,
    USB_CTRL_STAGE_DATA_IN,
    USB_CTRL_STAGE_STATUS_IN,
    USB_CTRL_STAGE_STATUS_OUT,
} ctrl_stage_t;

typedef enum {
    USBD_EVENT_RESET    = (1U << 0),
    USBD_EVENT_SOF      = (1U << 1),
    USBD_EVENT_SUSPEND  = (1U << 2),
    USBD_EVENT_RESUME   = (1U << 3),
    USBD_EVENT_WAKEUP   = (1U << 4),
    USBD_EVENT_EP_CMPLT = (1U << 5),
    USBD_EVENT_SETUP    = (1U << 6),
    USBD_EVENT_ERROR    = (1U << 7),
} usbd_event_t;

typedef struct ctrl_ep_     ctrl_ep_t;
typedef struct dcd_driver_  dcd_driver_t;
typedef struct usbd_handle_ usbd_handle_t;

typedef void (*usbd_endpoint_cb)(usbd_handle_t* handle, usbd_event_t event, uint8_t epaddr);
typedef void (*usbd_request_cb)(usbd_handle_t* handle, usb_ctrl_req_t* req);

// typedef struct usbd_handle usbd_handle_t;
// typedef void (*usbd_request_cb)(usbd_handle_t* handle, usb_ctrl_req_t* req);

/* ---- Application callbacks ---- */

/**
 * @brief USB driver initialization callback.
 * 
 * This callback is called when the USB device is mounted.
 * 
 * @param handle Pointer to the USB handle structure.
 */
typedef void (*usbd_init_cb)(usbd_handle_t *handle);

/**
 * @brief USB driver unmount callback.
 * 
 * This callback is called when the USB device is unmounted.
 * 
 * @param handle Pointer to the USB handle structure.
 */
typedef void (*usbd_deinit_cb)(usbd_handle_t *handle);

/**
 * @brief USB driver set configuration callback.
 * 
 * This callback is called when a SET_CONFIGURATION request is received
 * and the configuration value is > 0. If the configuration value is 0, 
 * the device is deconfigured and usbd_deinit_cb callback is invoked instead.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param config Configuration number.
 * 
 * @return true if the configuration was set successfully, false otherwise.
 */
typedef bool (*usbd_set_config_cb)(usbd_handle_t *handle, uint8_t config);

/**
 * @brief USB driver configuration complete callback.
 * 
 * This callback is called when the device is configured and
 * the SET_CONFIGURATION request is complete.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param config Configuration number.
 */
typedef void (*usbd_configured_cb)(usbd_handle_t *handle, uint8_t config);

/**
 * @brief USB driver get descriptor callback.
 * 
 * This callback is invoked when a GET_DESCRIPTOR request is 
 * received for any recipient, device, interface, endpoint, etc.
 * Respond to the request by calling usbd_send_ctrl_resp.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param request Pointer to the control request structure.
 * 
 * @return true if the descriptor was handled, false otherwise.
 */
typedef bool (*usbd_get_desc_cb)(usbd_handle_t *handle, const usb_ctrl_req_t *request);

/**
 * @brief USB driver control transfer callback.
 * 
 * This callback is invoked when a control transfer is received and
 * not handled internally or by usbd_get_desc_cb. To respond to a 
 * device-to-host request, call usbd_send_ctrl_resp. Only basic 
 * requests, like SET_ADDRESS, endpoint SET_FEATURE, etc. 
 * are handled internally.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param request Pointer to the control request structure.
 * 
 * @return true if the request was handled, false otherwise.
 */
typedef bool (*usbd_ctrl_xfer_cb)(usbd_handle_t *handle, const usb_ctrl_req_t *request);

/**
 * @brief USB driver endpoint transfer complete callback.
 * 
 * This callback is called when an endpoint transfer is complete.
 * 
 * @param handle Pointer to the USB device structure.
 * @param epaddr Endpoint address.
 */
typedef void (*usbd_ep_xfer_cb)(usbd_handle_t *handle, uint8_t epaddr);

typedef struct {   
    usbd_init_cb        init_cb;            /* Device init callback. */
    usbd_deinit_cb      deinit_cb;          /* Device deinit callback. */
    usbd_get_desc_cb    get_desc_cb;        /* Get descriptor callback. */
    usbd_set_config_cb  set_config_cb;      /* Set configuration request callback. */
    usbd_configured_cb  configured_cb;      /* Configuration complete callback, device mounted. */
    usbd_ctrl_xfer_cb   ctrl_xfer_cb;       /* Control transfer callback. Return true if request handled. */
    usbd_ep_xfer_cb     ep_xfer_cb;         /* Non-control endpoint transfer complete callback. */
} usbd_driver_t;

/* ---- USBD Hardware Types ---- */

typedef enum {
    USBD_HW_USB = 0, /* Native USB hardware. */
    USBD_HW_PIO,     /* Emulated USB via PIO. */
} usbd_hw_type_t;

typedef struct ctrl_ep_ {
    ctrl_stage_t    stage;
    usbd_request_cb complete_cb;

    uint8_t     tx_buf[USBD_ENUMERATION_SIZE] __attribute__((aligned(4)));
    uint16_t    tx_idx;
    uint16_t    tx_len;
    
    uint8_t     rx_buf[USBD_ENUMERATION_SIZE] __attribute__((aligned(4)));
    uint16_t    rx_idx;
    uint16_t    rx_len;
} ctrl_ep_t;

typedef struct usbd_handle_ {
    uint8_t             port;

    usbd_state_t        state;
    usbd_hw_type_t      hw_type;
    
    usbd_driver_t       app_driver;
    const dcd_driver_t* dcd_driver;

    uint8_t             config_num;
    usbd_endpoint_cb    endpoint_cb[USBD_ENDPOINTS_MAX];
    uint16_t            ctrl_ep_size;
    ctrl_ep_t           ctrl_ep;
    
    uint8_t             desc_serial_buf[USBD_SERIAL_BUF_SIZE] __attribute__((aligned(2)));
} usbd_handle_t;

/* ---- USBD API ---- */

/**
 * @brief Initialize the USB stack and hardware.
 *
 * @param hw_type Type of USB hardware to use (USB or PIO).
 * @param driver Pointer to the USB driver structure.
 * @param ep0_size Size of the control endpoint (EP0), which
 *                 must match bMaxPacketSize0 in the device descriptor.
 *
 * @return Pointer to an assigned device handle or NULL on failure.
 * 
 * @note If hw_type == USBD_HW_PIO, this function can used 
 *       USBD_DEVICES_MAX times to initialize multiple USB 
 *       devices (if emulating a hub via PIO). If using a hub,
 *       the hub must be the first PIO device initialized.
 */
usbd_handle_t* usbd_init(usbd_hw_type_t hw_type, const usbd_driver_t *driver, uint8_t ep0_size);

/**
 * @brief Process USB events.
 *
 * Function should be called in application's
 * main loop to process each USB device's events.
 */
void usbd_task(void);

/**
 * @brief Connect or disconnect the USB device.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param connect true to connect the device, false to disconnect it.
 */
void usbd_set_connected(usbd_handle_t* handle, bool connect);

/**
 * @brief Open a USB endpoint.
 * 
 * Function will open a USB endpoint with the specified attributes.
 * Should be used in set_config callback to open endpoints.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param epaddr Endpoint address.
 * @param epattr Endpoint attributes.
 * @param epsize Endpoint size.
 * 
 * @return true if the endpoint was opened successfully, false otherwise.
 */
bool usbd_ep_open(usbd_handle_t* handle, uint8_t epaddr, uint8_t epattr, uint16_t epsize);

/**
 * @brief Attempt to open all endpoints in a configuration descriptor.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param desc_config Pointer to the configuration descriptor.
 * 
 * @return true if all endpoints were opened successfully, false otherwise.
 */
bool usbd_configure_all_eps(usbd_handle_t* handle, const void* desc_config);

/**
 * @brief Queue a control response to the host.
 *
 * Queue a control response to the host.
 * Function will only return true during the DATA_IN stage
 * of a control request. Input buffer is immediately copied
 * and does not need to be available after calling.
 *
 * @param handle Pointer to the USB handle structure.
 * @param buffer Pointer to the data buffer to send.
 * @param len Length of the data to send.
 * @param complete_cb Callback function invoked when the 
 *                    transfer is complete, can be NULL.
 * 
 * @return true if the response was queued successfully, false otherwise.
 */
bool usbd_send_ctrl_resp(usbd_handle_t* handle, const void *buffer, uint16_t len, usbd_request_cb complete_cb);

/**
 * @brief Flush an endpoint and abort any queued transfer.
 *
 * Function will abort/clear any queued IN/OUT transfer.
 *
 * @param handle Pointer to the USB handle structure.
 * @param epaddr Endpoint address.
 */
void usbd_ep_flush(usbd_handle_t* handle, uint8_t epaddr);

/**
 * @brief Check if an endpoint is ready for transfer.
 *
 * Function will return true if the endpoint has no pending 
 * IN/OUT tranfer.
 *
 * @param handle Pointer to the USB handle structure.
 * @param epaddr Endpoint address.
 *
 * @return true if the endpoint has no pending tranfer, 
 *         false otherwise.
 */
bool usbd_ep_ready(usbd_handle_t* handle, uint8_t epaddr);

/**
 * @brief Write data to an endpoint.
 * 
 * Function will write data to the specified endpoint.
 * Input buffer is copied immediately and does not need to
 * be available after calling.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param epaddr Endpoint address.
 * @param buffer Pointer to the data buffer to send.
 * @param len Length of the data to send.
 * 
 * @return Number of bytes written or -1 on failure.
 */
int32_t usbd_ep_write(usbd_handle_t* handle, uint8_t epaddr, const void *buffer, uint16_t len);

/**
 * @brief Read data from an endpoint.
 * 
 * Function will read data from the specified endpoint.
 * 
 * @param handle Pointer to the USB handle structure.
 * @param epaddr Endpoint address.
 * @param buffer Pointer to the buffer to store the received data.
 * @param len Length of the data to read.
 * 
 * @return Number of bytes read or -1 on failure.
 */
int32_t usbd_ep_read(usbd_handle_t* handle, uint8_t epaddr, void *buffer, uint16_t len);

/**
 * @brief Reset a USB device.
 *
 * Function will close all endpoints and reset the control endpoint.
 * 
 * @param handle Pointer to the USB handle structure.
 * 
 * @note This function is intended for use on a hub's downstream 
 *       devices in response to a reset feature request. Use otherwise
 *       is not recommended.
 */
void usbd_reset_device(usbd_handle_t* handle);

/**
 * @brief Get the USB serial number string descriptor.
 * 
 * This function will return a pointer to a serial number string
 * descriptor. The descriptor is generated from the unique
 * board ID and port number of the handle.
 * 
 * @param handle Pointer to the USB handle structure.
 * 
 * @return Pointer to the serial number string descriptor.
 */
const usb_desc_string_t* usbd_get_desc_string_serial(usbd_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif /* USBD_DEVICES_MAX */