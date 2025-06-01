#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_MSC_SUBCLASS_RBC                    0x01 // Reduced Block Commands (RBC) T10 Project 1240-D
#define USB_MSC_SUBCLASS_SFF_MMC                0x02 // SFF-8020i, MMC-2 (ATAPI). Typically used by a CD/DVD device
#define USB_MSC_SUBCLASS_QIC                    0x03 // QIC-157. Typically used by a tape device
#define USB_MSC_SUBCLASS_UFI                    0x04 // UFI. Typically used by Floppy Disk Drive (FDD) device
#define USB_MSC_SUBCLASS_SFF                    0x05 // SFF-8070i. Can be used by Floppy Disk Drive (FDD) device
#define USB_MSC_SUBCLASS_SCSI                   0x06 // SCSI transparent command set

#define USB_REQ_MSC_GET_MAX_LUN                 0xFE
#define USB_REQ_MSC_RESET                       0xFF

#define USB_MSC_CBW_SIGNATURE                   ((uint32_t)0x43425355)
#define USB_MSC_CSW_SIGNATURE                   ((uint32_t)0x53425355)

#define USB_MSC_SCSI_CMD_TEST_UNIT_READY                0x00 // The SCSI Test Unit Ready command is used to determine if a device is ready to transfer data (read/write), i.e. if a disk has spun up, if a tape is loaded and ready etc. The device does not perform a self-test operation.
#define USB_MSC_SCSI_CMD_INQUIRY                        0x12 // The SCSI Inquiry command is used to obtain basic information from a target device.
#define USB_MSC_SCSI_CMD_MODE_SELECT_6                  0x15 //  provides a means for the application client to specify medium, logical unit, or peripheral device parameters to the device server. Device servers that implement the MODE SELECT(6) command shall also implement the MODE SENSE(6) command. Application clients should issue MODE SENSE(6) prior to each MODE SELECT(6) to determine supported mode pages, page lengths, and other parameters.
#define USB_MSC_SCSI_CMD_MODE_SENSE_6                   0x1A // provides a means for a device server to report parameters to an application client. It is a complementary command to the MODE SELECT(6) command. Device servers that implement the MODE SENSE(6) command shall also implement the MODE SELECT(6) command.
#define USB_MSC_SCSI_CMD_START_STOP_UNIT                0x1B
#define USB_MSC_SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL   0x1E
#define USB_MSC_SCSI_CMD_READ_CAPACITY_10               0x25 // The SCSI Read Capacity command is used to obtain data capacity information from a target device.
#define USB_MSC_SCSI_CMD_REQUEST_SENSE                  0x03 // The SCSI Request Sense command is part of the SCSI computer protocol standard. This command is used to obtain sense data -- status/error information -- from a target device.
#define USB_MSC_SCSI_CMD_READ_FORMAT_CAPACITY           0x23 // The command allows the Host to request a list of the possible format capacities for an installed writable media. This command also has the capability to report the writable capacity for a media when it is installed
#define USB_MSC_SCSI_CMD_READ_10                        0x28 // The READ (10) command requests that the device server read the specified logical block(s) and transfer them to the data-in buffer.
#define USB_MSC_SCSI_CMD_WRITE_10                       0x2A // The WRITE (10) command requests that the device server transfer the specified logical block(s) from the data-out buffer and write them.

#define USB_MSC_CSW_STATUS_PASSED       0x00
#define USB_MSC_CSW_STATUS_FAILED       0x01
#define USB_MSC_CSW_STATUS_PHASE_ERROR  0x02

typedef struct __attribute__((packed)) {
    uint32_t signature;   // Signature that helps identify this data packet as a CBW. The signature field shall contain the value 43425355h (little endian), indicating a CBW.
    uint32_t tag;         // Tag sent by the host. The device shall echo the contents of this field back to the host in the dCSWTagfield of the associated CSW. The dCSWTagpositively associates a CSW with the corresponding CBW.
    uint32_t total_bytes; // The number of bytes of data that the host expects to transfer on the Bulk-In or Bulk-Out endpoint (as indicated by the Direction bit) during the execution of this command. If this field is zero, the device and the host shall transfer no data between the CBW and the associated CSW, and the device shall ignore the value of the Direction bit in bmCBWFlags.
    uint8_t dir;          // Bit 7 of this field define transfer direction \n - 0 : Data-Out from host to the device. \n - 1 : Data-In from the device to the host.
    uint8_t lun;          // The device Logical Unit Number (LUN) to which the command block is being sent. For devices that support multiple LUNs, the host shall place into this field the LUN to which this command block is addressed. Otherwise, the host shall set this field to zero.
    uint8_t cmd_len;      // The valid length of the CBWCBin bytes. This defines the valid length of the command block. The only legal values are 1 through 16
    uint8_t command[16];  // The command block to be executed by the device. The device shall interpret the first cmd_len bytes in this field as a command block
} usb_msc_cbw_t;

typedef struct __attribute__((packed)) {
    uint32_t signature; // Signature that helps identify this data packet as a CSW. The signature field shall contain the value 53425355h (little endian), indicating CSW.
    uint32_t tag; // The device shall set this field to the value received in the dCBWTag of the associated CBW.
    uint32_t data_residue; // For Data-Out the device shall report in the dCSWDataResidue the difference between the amount of data expected as stated in the dCBWDataTransferLength, and the actual amount of data processed by the device. For Data-In the device shall report in the dCSWDataResiduethe difference between the amount of data expected as stated in the dCBWDataTransferLengthand the actual amount of relevant data sent by the device
    uint8_t  status; // indicates the success or failure of the command. Values from \ref msc_csw_status_t
} usb_msc_csw_t;

#ifdef __cplusplus
}
#endif
