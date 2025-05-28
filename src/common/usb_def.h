#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USB Class */

#define USB_CLASS_UNSPECIFIED           0x00    /* Unspecified class code. */
#define USB_CLASS_AUDIO                 0x01    /* Audio class code. */
#define USB_CLASS_CDC                   0x02    /* Communication class code. */
#define USB_CLASS_HID                   0x03    /* HID class code. */
#define USB_CLASS_PHYSICAL              0x05    /* Physical class code. */
#define USB_CLASS_IMAGE                 0x06    /* Image class code. */
#define USB_CLASS_PRINTER               0x07    /* Printer class code. */
#define USB_CLASS_MSC                   0x08    /* Mass storage class code. */
#define USB_CLASS_HUB                   0x09    /* Hub class code. */
#define USB_CLASS_CDC_DATA              0x0A    /* CDC data class code. */
#define USB_CLASS_SMARTCARD             0x0B    /* Smart card class code. */
#define USB_CLASS_CONTENT_SECURITY      0x0D    /* Content security class code. */
#define USB_CLASS_VIDEO                 0x0E    /* Video class code. */
#define USB_CLASS_PERSONAL_HEALTHCARE   0x0F    /* Personal healthcare class code. */
#define USB_CLASS_AUDIO_VIDEO           0x10    /* Audio video class code. */
#define USB_CLASS_BILLBOARD             0x11    /* Billboard class code. */
#define USB_CLASS_TYPE_C_BRIDGE         0x12    /* Type-C bridge class code. */
#define USB_CLASS_DIAGNOSTIC            0xDC    /* Diagnostic device class code. */
#define USB_CLASS_WIRELESS_CONTROLLER   0xE0    /* Wireless controller class code. */
#define USB_CLASS_MISC                  0xEF    /* Miscellaneous class code. */
#define USB_CLASS_APPLICATION_SPECIFIC  0xFE    /* Application specific class code. */
#define USB_CLASS_VENDOR                0xFF    /* Vendor specific class code. */

/* USB Subclass */

#define USB_SUBCLASS_NONE       0x00    /* No subclass. */
#define USB_SUBCLASS_IAD        0x02    /* Subclass defined by interface */
#define USB_SUBCLASS_VENDOR     0xFF    /* Vendor subclass. */

/* USB Protocol */

#define USB_PROTOCOL_NONE       0x00    /* No protocol. */
#define USB_PROTOCOL_IAD        0x01    /* Protocol defined by interface */
#define USB_PROTOCOL_VENDOR     0xFF    /* Vendor protocol. */

/* USB BCD Version */

#define USB_BCD_VERSION_1_0    0x0100 /* USB 1.0 */
#define USB_BCD_VERSION_1_1    0x0110 /* USB 1.1 */
#define USB_BCD_VERSION_2_0    0x0200 /* USB 2.0 */
#define USB_BCD_VERSION_2_1    0x0210 /* USB 2.1 */
#define USB_BCD_VERSION_3_0    0x0300 /* USB 3.0 */
#define USB_BCD_VERSION_3_1    0x0310 /* USB 3.1 */
#define USB_BCD_VERSION_3_2    0x0320 /* USB 3.2 */

// USB Configuration Attributes (bmAttributes)
#define USB_ATTR_RESERVED          0x80    // Bit 7 must always be set (USB spec)
#define USB_ATTR_SELF_POWERED      0x40    // Bit 6: Self-powered device (not bus-powered)
#define USB_ATTR_BUS_POWERED       0x00    // Device is bus-powered (draws power from USB)
#define USB_ATTR_REMOTE_WAKEUP     0x20    // Bit 5: Device supports remote wakeup

/* USB EP */

#define USB_EP_DIR_OUT                  0x00    /* Host-to-device endpoint direction.*/
#define USB_EP_DIR_IN                   0x80    /* Device-to-host endpoint direction.*/

#define USB_EP_TYPE_CONTROL             0x00    /* Control endpoint.*/
#define USB_EP_TYPE_ISOCHRONUS          0x01    /* Isochronous endpoint.*/
#define USB_EP_TYPE_BULK                0x02    /* Bbulk endpoint.*/
#define USB_EP_TYPE_INTERRUPT           0x03    /* Interrupt endpoint.*/
#define USB_EP_TYPE_DBLBUF              0x04    /* Doublebuffered endpoint.*/

#define USB_EP_ATTR_NO_SYNC             0x00    /* No synchronization.*/
#define USB_EP_ATTR_ASYNC               0x04    /* Asynchronous endpoint.*/
#define USB_EP_ATTR_ADAPTIVE            0x08    /* Adaptive endpoint.*/
#define USB_EP_ATTR_SYNC                0x0C    /* Synchronous endpoint.*/

/* USB Request */

/* bmRequestType field */
#define USB_REQ_DIR_Msk                 (1U << 7)   /* Request direction mask.*/
#define USB_REQ_DIR_HOSTTODEV           (0U << 7)   /* Request direction is HOST to DEVICE.*/
#define USB_REQ_DIR_DEVTOHOST           (1U << 7)   /* Request direction is DEVICE to HOST.*/

#define USB_REQ_TYPE_Msk                (3U << 5)   /* Request type mask.*/
#define USB_REQ_TYPE_STANDARD           (0U << 5)   /* Standard request.*/
#define USB_REQ_TYPE_CLASS              (1U << 5)   /* Class specified request.*/
#define USB_REQ_TYPE_VENDOR             (2U << 5)   /* Vendor specified request.*/

#define USB_REQ_RECIP_Msk               (3U << 0)   /* Request recipient mask.*/
#define USB_REQ_RECIP_DEVICE            (0U << 0)   /* Request to device.*/
#define USB_REQ_RECIP_INTERFACE         (1U << 0)   /* Request to interface.*/
#define USB_REQ_RECIP_ENDPOINT          (2U << 0)   /* Request to endpoint.*/
#define USB_REQ_RECIP_OTHER             (3U << 0)   /* Other request.*/

/* USB Standard descriptor types */
#define USB_DTYPE_DEVICE                0x01    /* Device descriptor.*/
#define USB_DTYPE_CONFIGURATION         0x02    /* Configuration descriptor.*/
#define USB_DTYPE_STRING                0x03    /* String descriptor.*/
#define USB_DTYPE_INTERFACE             0x04    /* Interface descriptor.*/
#define USB_DTYPE_ENDPOINT              0x05    /* Endpoint  descriptor.*/
#define USB_DTYPE_QUALIFIER             0x06    /* Qualifier descriptor.*/
#define USB_DTYPE_OTHER                 0x07    /* Descriptor is of other type. */
#define USB_DTYPE_INTERFACEPOWER        0x08    /* Interface power descriptor. */
#define USB_DTYPE_OTG                   0x09    /* OTG descriptor.*/
#define USB_DTYPE_DEBUG                 0x0A    /* Debug descriptor.*/
#define USB_DTYPE_IAD                   0x0B    /* Interface association descriptor.*/
#define USB_DTYPE_CS_INTERFACE          0x24    /* Class specific interface descriptor.*/
#define USB_DTYPE_CS_ENDPOINT           0x25    /* Class specific endpoint descriptor.*/
#define USB_DTYPE_HUB                   0x29    /* Hub descriptor.*/

/* Special string descriptor indexes */
#define NO_DESCRIPTOR                   0x00    /* String descriptor doesn't exists in the device.*/
#define INTSERIALNO_DESCRIPTOR          0xFE    /* String descriptor is an internal serial number provided by hardware driver.*/

/* USB Standard requests */
#define USB_REQ_STD_GET_STATUS          0x00    /* Returns status for the specified recipient.*/
#define USB_REQ_STD_CLEAR_FEATURE       0x01    /* Used to clear or disable a specific feature.*/
#define USB_REQ_STD_SET_FEATURE         0x03    /* Used to set or enable a specific feature.*/
#define USB_REQ_STD_SET_ADDRESS         0x05    /* Sets the device address for all future device accesses.*/
#define USB_REQ_STD_GET_DESCRIPTOR      0x06    /* Returns the specified descriptor if the descriptor exists.*/
#define USB_REQ_STD_SET_DESCRIPTOR      0x07    /* This request is optional and may be used to update existing descriptors or new descriptors may be added.*/
#define USB_REQ_STD_GET_CONFIG          0x08    /* Returns the current device configuration value.*/
#define USB_REQ_STD_SET_CONFIG          0x09    /* Sets the device configuration.*/
#define USB_REQ_STD_GET_INTERFACE       0x0A    /* Returns the selected alternate setting for the specified interface.*/
#define USB_REQ_STD_SET_INTERFACE       0x0B    /* Allows the host to select an alternate setting for the specified interface.*/
#define USB_REQ_STD_SYNCH_FRAME         0x0C    /* Used to set and then report an endpoint's synchronization frame.*/

/* USB Feature selector */

#define USB_FEAT_ENDPOINT_HALT          0x00    /* Halt endpoint.*/
#define USB_FEAT_REMOTE_WKUP            0x01
#define USB_FEAT_TEST_MODE              0x02
#define USB_FEAT_DEBUG_MODE             0x06

/* Feature request values (set/clear (1U < value) bit in feature mask) */

#define USB_FEATURE_PORT_CONNECTION     0U
#define USB_FEATURE_PORT_ENABLE	        1U
#define USB_FEATURE_PORT_SUSPEND        2U
#define USB_FEATURE_PORT_OVERCURRENT    3U
#define USB_FEATURE_PORT_RESET	        4U

#define USB_FEATURE_PORT_POWER	        8U
#define USB_FEATURE_PORT_LOWSPEED	    9U
#define USB_FEATURE_PORT_HIGHSPEED	    10U
#define USB_FEATURE_PORT_TEST_MODE	    11U
#define USB_FEATURE_PORT_INDICATOR	    12U

#define USB_FEATURE_C_PORT_CONNECTION   16U
#define USB_FEATURE_C_PORT_ENABLE	    17U
#define USB_FEATURE_C_PORT_SUSPEND	    18U
#define USB_FEATURE_C_PORT_OVERCURRENT  19U
#define USB_FEATURE_C_PORT_RESET        20U

#define USB_FEATURE_PORT_Pos            0U
#define USB_FEATURE_PORT_Msk            ((uint32_t)0x0000FFFF)
#define USB_FEATURE_C_PORT_Pos          16U
#define USB_FEATURE_C_PORT_Msk          ((uint32_t)0xFFFF0000)

/* USB Data structures */

/* USB control request */
typedef struct __attribute__((packed)) {
    uint8_t     bmRequestType;  /* Characteristics of the specific request.*/
    uint8_t     bRequest;       /* This field specifies the particular request.*/
    uint16_t    wValue;         /* It is used to pass a parameter to the device, specific to the request.*/
    uint16_t    wIndex;         /* It is used to pass a parameter to the device, specific to the request.*/
    uint16_t    wLength;        /* This field specifies the length of the data transferred during the second phase of the control transfer.*/
    uint8_t     data[];         /* Data payload.*/
} usb_ctrl_req_t;

/* USB descriptor header */
typedef struct __attribute__((packed)) {
    uint8_t bLength;                /* Size of the descriptor, in bytes. */
    uint8_t bDescriptorType;        /* Type of the descriptor. */
} usb_desc_header_t;

/* USB device descriptor */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;               /* Size of the descriptor, in bytes.*/
    uint8_t  bDescriptorType;       /* USB_DTYPE_DEVICE Device descriptor.*/
    uint16_t bcdUSB;                /* BCD of the supported USB specification.*/
    uint8_t  bDeviceClass;          /* USB device class.*/
    uint8_t  bDeviceSubClass;       /* USB device subclass.*/
    uint8_t  bDeviceProtocol;       /* USB device protocol.*/
    uint8_t  bMaxPacketSize0;       /* Size of the control endpoint's bank in bytes.*/
    uint16_t idVendor;              /* Vendor ID for the USB product.*/
    uint16_t idProduct;             /* Unique product ID for the USB product.*/
    uint16_t bcdDevice;             /* Product release (version) number.*/
    uint8_t  iManufacturer;         /* String index for the manufacturer's name.*/
    uint8_t  iProduct;              /* String index for the product name/details.*/
    uint8_t  iSerialNumber;         /* String index for the product serial number.*/
    uint8_t  bNumConfigurations;    /* Total number of configurations supported by the device.*/
} usb_desc_device_t;

/* USB device configuration descriptor */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;               /* Size of the descriptor, in bytes.*/
    uint8_t  bDescriptorType;       /* Configuration descriptor.*/
    uint16_t wTotalLength;          /* Size of the configuration descriptor header, and all sub descriptors inside the configuration. */
    uint8_t  bNumInterfaces;        /* Total number of interfaces in the configuration.*/
    uint8_t  bConfigurationValue;   /* Configuration index of the current configuration.*/
    uint8_t  iConfiguration;        /* Index of a string descriptor describing the configuration.*/
    uint8_t  bmAttributes;          /* Configuration attributes. */
    uint8_t  bMaxPower;             /* Maximum power consumption of the device. USB_CFG_POWER_MA() macro.*/
} usb_desc_config_t;

/* USB interface descriptor */
typedef struct __attribute__((packed)) {
    uint8_t bLength;                /* Size of the descriptor, in bytes.*/
    uint8_t bDescriptorType;        /* Interface descriptor.*/
    uint8_t bInterfaceNumber;       /* Index of the interface in the current configuration.*/
    uint8_t bAlternateSetting;      /* Alternate setting for the interface number.*/
    uint8_t bNumEndpoints;          /* Total number of endpoints in the interface.*/
    uint8_t bInterfaceClass;        /* Interface class ID.*/
    uint8_t bInterfaceSubClass;     /* Interface subclass ID.*/
    uint8_t bInterfaceProtocol;     /* Interface protocol ID. */
    uint8_t iInterface;             /* Index of the string descriptor describing the interface. */
} usb_desc_itf_t;

/* USB endpoint descriptor */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;               /* Size of the descriptor, in bytes. */
    uint8_t  bDescriptorType;       /* Endpoint descriptor.*/
    uint8_t  bEndpointAddress;      /* Logical address of the endpoint within the device for the current configuration, including direction mask. */
    uint8_t  bmAttributes;          /* Endpoint attributes, USB_ENDPOINT_DEF. */
    uint16_t wMaxPacketSize;        /* Size of the endpoint bank, in bytes. This indicates the maximum packet size that the endpoint can receive at a time. */
    uint8_t  bInterval;             /* Polling interval in milliseconds for the endpoint if it is an INTERRUPT or ISOCHRONOUS type.*/
} usb_desc_endpoint_t;

/* USB string descriptor */
typedef struct __attribute__((packed, aligned(2))) {
    uint8_t  bLength;               /* Size of the descriptor, in bytes.*/
    uint8_t  bDescriptorType;       /* String descriptor type.*/
    uint16_t wString[];             /* String data, as unicode characters or array of USB_STD_LANGID codes. */
} usb_desc_string_t;

/* USB Interface Association descriptor */
typedef struct __attribute__((packed)) {
    uint8_t bLength;                /* Size of the descriptor, in bytes.*/
    uint8_t bDescriptorType;        /* IAD descriptor */
    uint8_t bFirstInterface;        /* Index of the first associated interface. */
    uint8_t bInterfaceCount;        /* Total number of associated interfaces. */
    uint8_t bFunctionClass;         /* Function class ID. */
    uint8_t bFunctionSubClass;      /* Function subclass ID. */
    uint8_t bFunctionProtocol;      /* Function protocol ID. */
    uint8_t iFunction;              /* Index of the string descriptor describing the
                                     * interface association. */
} usb_desc_iad_t;

#ifdef __cplusplus
}
#endif