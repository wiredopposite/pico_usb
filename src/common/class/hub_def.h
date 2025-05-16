#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USB HUB Subclass */

#define USB_SUBCLASS_HUB                        0x00    /* No subclass for hubs */

/* USB HUB Protocol */

#define USB_PROTOCOL_HUB_FULL_SPEED             0x00    /* Full-speed hub (USB 1.1) */
#define USB_PROTOCOL_HUB_HIGH_SPEED_SINGLE_TT   0x01    /* Hi-speed hub with single Transaction Translator (TT) (USB 2.0) */
#define USB_PROTOCOL_HUB_HIGH_SPEED_MULTI_TT    0x02    /* Hi-speed hub with multiple TTs (USB 2.0) */
#define USB_PROTOCOL_HUB_SUPER_SPEED            0x03    /* SuperSpeed hub (USB 3.0/3.1 Gen 1) */
#define USB_PROTOCOL_HUB_SUPER_SPEED_PLUS       0x04    /* SuperSpeedPlus hub (USB 3.1 Gen 2/USB 3.2) */

/* HUB Characteristics */

#define USB_HUB_CHAR_POWER_SWITCH_GANGED        (0U << 0)
#define USB_HUB_CHAR_POWER_SWITCH_INDIVIDUAL    (1U << 0)
#define USB_HUB_CHAR_POWER_SWITCH_NONE          (1U << 1)

#define USB_HUB_CHAR_COMPOUND_DEVICE            (1U << 2)

#define USB_HUB_CHAR_OVERCURR_PROTECT           (1U << 4)

#define USB_HUB_CHAR_THINKTIME_8_FS_BITS        (0U << 5)  // 0x00
#define USB_HUB_CHAR_THINKTIME_16_FS_BITS       (1U << 5)  // 0x20
#define USB_HUB_CHAR_THINKTIME_24_FS_BITS       (2U << 5)  // 0x40
#define USB_HUB_CHAR_THINKTIME_32_FS_BITS       (3U << 5)  // 0x60

#define USB_HUB_CHAR_PORT_INDICATORS            (1U << 7)

/* USB hub descriptor */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;               /* Size of the descriptor, in bytes.*/
    uint8_t  bDescriptorType;       /* Hub descriptor.*/
    uint8_t  bNumberOfPorts;        /* Number of downstream ports.*/
    uint16_t wHubCharacteristics;   /* Hub characteristics.*/
    uint8_t  bPowerOnToPowerGood;   /* Power on to power good time.*/
    uint8_t  bHubControlCurrent;    /* Hub control current.*/
    uint8_t  bRemoveAndPowerMask[]; /* Device removable and power mask.*/
} usb_desc_hub_t;

#ifdef __cplusplus
}
#endif