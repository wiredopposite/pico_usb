#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CDC Subclass Codes */

#define USB_SUBCLASS_CDC_DIRECT_LINE_CONTROL        0x01 /* Direct Line Control Model           [USBPSTN1.2] */
#define USB_SUBCLASS_CDC_ABSTRACT_CONTROL           0x02 /* Abstract Control Model              [USBPSTN1.2] */
#define USB_SUBCLASS_CDC_TELEPHONE_CONTROL          0x03 /* Telephone Control Model             [USBPSTN1.2] */
#define USB_SUBCLASS_CDC_MULTICHANNEL_CONTROL       0x04 /* Multi-Channel Control Model         [USBISDN1.2] */
#define USB_SUBCLASS_CDC_CAPI_CONTROL               0x05 /* CAPI Control Model                  [USBISDN1.2] */
#define USB_SUBCLASS_CDC_ETHERNET_CONTROL           0x06 /* Ethernet Networking Control Model   [USBECM1.2] */
#define USB_SUBCLASS_CDC_ATM_NETWORKING_CONTROL     0x07 /* ATM Networking Control Model        [USBATM1.2] */
#define USB_SUBCLASS_CDC_WIRELESS_HANDSET_CONTROL   0x08 /* Wireless Handset Control Model      [USBWMC1.1] */
#define USB_SUBCLASS_CDC_DEVICE_MANAGEMENT          0x09 /* Device Management                   [USBWMC1.1] */
#define USB_SUBCLASS_CDC_MOBILE_DIRECT_LINE         0x0A /* Mobile Direct Line Model            [USBWMC1.1] */
#define USB_SUBCLASS_CDC_OBEX                       0x0B /* OBEX                                [USBWMC1.1] */
#define USB_SUBCLASS_CDC_ETHERNET_EMULATION         0x0C /* Ethernet Emulation Model            [USBEEM1.0] */
#define USB_SUBCLASS_CDC_NETWORK_CONTROL            0x0D /* Network Control Model               [USBNCM1.0] */

/* CDC Protocols */

#define USB_PROTOCOL_CDC_NONE                           0x00 /* No specific protocol                        */
#define USB_PROTOCOL_CDC_ATCOMMAND                      0x01 /* AT Commands: V.250 etc                      */
#define USB_PROTOCOL_CDC_ATCOMMAND_PCCA_101             0x02 /* AT Commands defined by PCCA-101             */
#define USB_PROTOCOL_CDC_ATCOMMAND_PCCA_101_AND_ANNEXO  0x03 /* AT Commands defined by PCCA-101 & Annex O   */
#define USB_PROTOCOL_CDC_ATCOMMAND_GSM_707              0x04 /* AT Commands defined by GSM 07.07            */
#define USB_PROTOCOL_CDC_ATCOMMAND_3GPP_27007           0x05 /* AT Commands defined by 3GPP 27.007          */
#define USB_PROTOCOL_CDC_ATCOMMAND_CDMA                 0x06 /* AT Commands defined by TIA for CDMA         */
#define USB_PROTOCOL_CDC_ETHERNET_EMULATION             0x07 /* Ethernet Emulation Model                    */

/* CDC Descriptor Types */

#define USB_DTYPE_CS_INTERFACE           0x24    /* Class-specific interface descriptor type */
#define USB_DTYPE_CS_ENDPOINT            0x25    /* Class-specific endpoint descriptor type */

/* CDC Functional Descriptor Subtypes */

#define USB_DTYPE_CDC_HEADER           0x00    /* Header functional descriptor */
#define USB_DTYPE_CDC_CALL_MGMT        0x01    /* Call Management functional descriptor */
#define USB_DTYPE_CDC_ACM              0x02    /* Abstract Control Management */
#define USB_DTYPE_CDC_UNION            0x06    /* Union functional descriptor */

/* CDC Class-Specific Requests */

#define USB_REQ_CDC_SEND_ENCAPSULATED_COMMAND                     0x00 
#define USB_REQ_CDC_GET_ENCAPSULATED_RESPONSE                     0x01 
#define USB_REQ_CDC_SET_COMM_FEATURE                              0x02
#define USB_REQ_CDC_GET_COMM_FEATURE                              0x03
#define USB_REQ_CDC_CLEAR_COMM_FEATURE                            0x04

#define USB_REQ_CDC_SET_AUX_LINE_STATE                            0x10
#define USB_REQ_CDC_SET_HOOK_STATE                                0x11
#define USB_REQ_CDC_PULSE_SETUP                                   0x12
#define USB_REQ_CDC_SEND_PULSE                                    0x13
#define USB_REQ_CDC_SET_PULSE_TIME                                0x14
#define USB_REQ_CDC_RING_AUX_JACK                                 0x15

#define USB_REQ_CDC_SET_LINE_CODING                               0x20
#define USB_REQ_CDC_GET_LINE_CODING                               0x21
#define USB_REQ_CDC_SET_CONTROL_LINE_STATE                        0x22
#define USB_REQ_CDC_SEND_BREAK                                    0x23

#define USB_REQ_CDC_SET_RINGER_PARMS                              0x30
#define USB_REQ_CDC_GET_RINGER_PARMS                              0x31
#define USB_REQ_CDC_SET_OPERATION_PARMS                           0x32
#define USB_REQ_CDC_GET_OPERATION_PARMS                           0x33
#define USB_REQ_CDC_SET_LINE_PARMS                                0x34
#define USB_REQ_CDC_GET_LINE_PARMS                                0x35
#define USB_REQ_CDC_DIAL_DIGITS                                   0x36
#define USB_REQ_CDC_SET_UNIT_PARAMETER                            0x37
#define USB_REQ_CDC_GET_UNIT_PARAMETER                            0x38
#define USB_REQ_CDC_CLEAR_UNIT_PARAMETER                          0x39
#define USB_REQ_CDC_GET_PROFILE                                   0x3A

#define USB_REQ_CDC_SET_ETHERNET_MULTICAST_FILTERS                0x40
#define USB_REQ_CDC_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER  0x41
#define USB_REQ_CDC_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER  0x42
#define USB_REQ_CDC_SET_ETHERNET_PACKET_FILTER                    0x43
#define USB_REQ_CDC_GET_ETHERNET_STATISTIC                        0x44

#define USB_REQ_CDC_SET_ATM_DATA_FORMAT                           0x50
#define USB_REQ_CDC_GET_ATM_DEVICE_STATISTICS                     0x51
#define USB_REQ_CDC_SET_ATM_DEFAULT_VC                            0x52
#define USB_REQ_CDC_GET_ATM_VC_STATISTICS                         0x53

#define USB_REQ_CDC_MDLM_SEMANTIC_MODEL                           0x60

/* CDC Notifications */

#define USB_NOTIF_CDC_NETWORK_CONNECTION                0x00
#define USB_NOTIF_CDC_RESPONSE_AVAILABLE                0x01
#define USB_NOTIF_CDC_AUX_JACK_HOOK_STATE               0x08
#define USB_NOTIF_CDC_RING_DETECT                       0x09
#define USB_NOTIF_CDC_SERIAL_STATE                      0x20
#define USB_NOTIF_CDC_CALL_STATE_CHANGE                 0x28
#define USB_NOTIF_CDC_LINE_STATE_CHANGE                 0x29
#define USB_NOTIF_CDC_CONNECTION_SPEED_CHANGE           0x2A
#define USB_NOTIF_CDC_MDLM_SEMANTIC_MODEL_NOTIFICATION  0x40

/* Control Line State bits */

#define USB_CDC_CONTROL_LINE_DTR     0x01    /* DTR signal */
#define USB_CDC_CONTROL_LINE_RTS     0x02    /* RTS signal */

/* Line coding structure */

#define USB_CDC_1_STOP_BITS             0x00    /* 1 stop bit.*/
#define USB_CDC_1_5_STOP_BITS           0x01    /* 1.5 stop bits.*/
#define USB_CDC_2_STOP_BITS             0x02    /* 2 stop bits.*/
#define USB_CDC_NO_PARITY               0x00    /* NO parity bit.*/
#define USB_CDC_ODD_PARITY              0x01    /* ODD parity bit.*/
#define USB_CDC_EVEN_PARITY             0x02    /* EVEN parity bit.*/
#define USB_CDC_MARK_PARITY             0x03    /* patity is MARK.*/
#define USB_CDC_SPACE_PARITY            0x04    /* patity is SPACE.*/

/* ACM Functional Descriptor bmCapabilities bits */

#define USB_CDC_ACM_CAP_COMM_FEATURES    0x01  /* Supports CommFeature requests */
#define USB_CDC_ACM_CAP_LINE_CODING      0x02  /* Supports LineCoding/ControlState */
#define USB_CDC_ACM_CAP_SEND_BREAK       0x04  /* Supports SendBreak */
#define USB_CDC_ACM_CAP_NETWORK_CONN     0x08  /* Supports NetworkConnection notification */

/* Call Management Functional Descriptor bmCapabilities bits */

#define USB_CDC_CALL_MGMT_CAP_CALL_MGMT  0x01  /* Device handles call management itself */
#define USB_CDC_CALL_MGMT_CAP_DATA_ITF   0x02  /* Device can send/receive call management over Data interface */

typedef struct __attribute__((packed)) {
    uint32_t dwDTERate;          /* Data terminal rate in bits per second */
    uint8_t  bCharFormat;        /* Stop bits: 0=1 bit, 1=1.5 bits, 2=2 bits */
    uint8_t  bParityType;        /* Parity: 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
    uint8_t  bDataBits;          /* Data bits (5, 6, 7, 8, or 16) */
} usb_cdc_line_coding_t;

typedef struct __attribute__((packed)) {
    uint8_t     bFunctionLength;    /* Size of this functional descriptor, in bytes.*/
    uint8_t     bDescriptorType;    /* CS_INTERFACE descriptor type.*/
    uint8_t     bDescriptorSubType; /* Call Management functional descriptor subtype.*/
    uint8_t     bmCapabilities;     /* The call management capabilities that this
                                     * configuration supports.*/
    uint8_t     bDataInterface;     /* Interface number of Data Class interface optionally
                                     * used for call management.*/
} usb_cdc_desc_call_mgmt_t;

typedef struct __attribute__((packed)) {
    uint8_t     bFunctionLength;    /* Size of this descriptor in bytes.*/
    uint8_t     bDescriptorType;    /* CS_INTERFACE descriptor type.*/
    uint8_t     bDescriptorSubType; /* Header functional descriptor subtype.*/
    uint16_t    bcdCDC;             /* USB CDC Specification release number in BCD.*/
} usb_cdc_desc_header_t;

typedef struct __attribute__((packed)) {
    uint8_t     bFunctionLength;    /* Size of this functional descriptor, in bytes.*/
    uint8_t     bDescriptorType;    /* CS_INTERFACE descriptor type.*/
    uint8_t     bDescriptorSubType; /* Abstract Control Management functional descriptor subtype.*/
    uint8_t     bmCapabilities;     /* The capabilities that this configuration supports.*/
} usb_cdc_desc_acm_t;

typedef struct __attribute__((packed)) {
    uint8_t     bFunctionLength;    /* Size of this functional descriptor, in bytes.*/
    uint8_t     bDescriptorType;    /* CS_INTERFACE descriptor type.*/
    uint8_t     bDescriptorSubType; /* Union Functional Descriptor.*/
    uint8_t     bMasterInterface0;  /* The interface number of the CDC interface designated
                                     * as the master or controlling interface for the union.*/
    uint8_t     bSlaveInterface0;   /* Interface number of first slave or associated interface
                                     * in the union.*/
    /* ... and there could be other slave interfaces */
} usb_cdc_desc_union_t;

#ifdef __cplusplus
}
#endif