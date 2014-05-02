

#ifndef _USBCMDIO_INCLUDED
#define _USBCMDIO_INCLUDED

#include <stdint.h>
#include "buffer.h"	//for readings struct

#ifdef GLOBAL_USBCMDIO
#define USBCMDIOGLOBAL
#define USBCMDIOPRESET(A) = (A)
#else
#define USBCMDIOPRESET(A)
#ifdef __cplusplus
#define USBCMDIOGLOBAL extern "C"
#else
#define USBCMDIOGLOBAL extern
#endif  /*__cplusplus*/
#endif                          /*GLOBAL_USBCMDIO */

#ifdef __cplusplus
extern "C" {
#endif

// Packet type
typedef enum {
  CMD_ACK = 0,
  CMD_NAK,
  CMD_RESET,
  CMD_ID,
  CMD_WRITE_REG,
  CMD_READ_REG,
  CMD_ECHO,
  CMD_SSN,        // silicon serial number
  CMD_UID,        // UID input for license
  CMD_OPT,        // licensed option(s) command(s)
  CMD_ISN,        // set/get instrument serial number
  CMD_DIAG,       // diagnostic self-test cmd, result-string
  CMD_ADC,	  // Ryan, One ADC reading output #12 or type = 0x0C
  CMD_ADC_ALL,	  // Ryan, All ADC readings get sent #13 or type = 0x0D
  CMD_ADC_TEST2,  // Ryan, load a packet with more then 1 reading and send, 0x0F
} pkttype_t;

// ACK, NAK, and RESET have payload length of 0
// ID has payload length of 0 for host->device direction
// ID gets a payload_id_response_t in the device->host direction
// WRITE_REG has a payload host->device, device->host is an ACK or NAK
// READ_REG host->device sends just the reg addr and number of reg's
// READ_REG device->host returns reg addr, number of reg's followed by contents
// READ_REG, RESET, and ID can return ACK or NAK
// I suppose ACK will return ACK, and NAK will return ACK
// ACK and NAK should probably return status/error codes in a struct
// ADC gets a payload of payload_adc_t
// 


typedef struct {            // size description
  uint8_t  productID;       // 1    start at 1
  uint8_t  protocolVersion; // 1    Must be 0x01 !!
  uint8_t  fwRev_major;     // 1    start at 00
  uint8_t  fwRev_minor;     // 1    start at 01
  uint16_t fwRev_build;     // 2    derive from Jenkins, start at 104
                            //---
                            // 6    size to this point
                            //
  uint8_t  bld_sha_len;     // 1    value = 8: 7 for SHA1, 1 for null-term
  uint8_t  bld_sha[8];      // 15   size to this point
  uint8_t  bld_info_len;    // 1    set dynamically
  uint8_t  bld_info [220];  // up to 220 chars, null-term
} payload_id_response_t;

typedef struct {
  uint8_t regAddr; //Start register address
  uint8_t numReg;  //number of registers. Must be >= 1
  uint8_t values[248];
} payload_reg_io_t;

typedef struct {  // SSN: silicon serial number
  uint8_t  ssn_cnt;       // STM32 = 3 (96-bit), NXP = 4 (128-bit)
  uint8_t  dummy2;
  uint8_t  dummy3;
  uint8_t  dummy4;        // alignment test...
  uint32_t ssn_values[4]; // zero-fill MS int32
} payload_ssn_t;
    
typedef struct {  // UID: unique ID
  uint8_t  uid_values[6]; // 48-bits
} payload_uid_t;

typedef struct {	//Determine how to end the packet, AKA Null terminate, actully might not be needed.
//  uint8_t numReadings;  //number of readings. Must be >= 1. Max of 254 bytes in a usb packet, header =4 bytes, payload=6 bytes each
//  uint8_t values[248];// max readings = (254 - 4)/payload = 41 readings.
  uint16_t current;
  uint16_t voltage;
  uint8_t DUTstate;
} payload_adc_t;//Ryan payload type for our readings

typedef struct {	//Determine how to end the packet, AKA Null terminate, actully might not be needed.
//  uint8_t numReadings;  //number of readings. Must be >= 1. Max of 254 bytes in a usb packet, header =4 bytes, payload=6 bytes each
//  uint8_t values[248];// max readings = (254 - 4)/payload = 41 readings.
  readings values[24]; 	//# of readings will be indicated in the length field, to calculate: (length-4)/5 = #readings
} payload_adc_all_t;//Ryan payload type for our readings in bulk

    
typedef struct {	//These are the structs to change to be our ADC data
  uint8_t length;    // Number of bytes in this packet: len + type + cksum(2-bytes) so min = 4 bytes
  uint8_t type;      // one byte or the command
  uint16_t checksum; // two bytes, little endian
  union { /*a union is like a struct, but only one of the variables can have a value at time*/
    uint8_t asBytes[250];
    payload_id_response_t id_resp;
    payload_reg_io_t reg_io;
    payload_ssn_t    ssn_resp;
    payload_uid_t    uid_resp;
    payload_adc_t    adc_resp; //Ryan 1 adc reading, can be only 1 reading
    payload_adc_all_t adc_all_resp; //Ryan multi adc readings, can be 1 or more
  } payload;
} usb_packet_t;

#define USB_PKT_MIN_HEADER_SZ 4  // len + cmd/type + cksum, NO other data
 
// Simple, speedy 8 bit checksum
typedef struct {
    uint16_t Checksum1;
    uint16_t Checksum2;
} FLETCHER_CHECKSUM;

/* Compute fletcher checksum in a streaming fashion */
#ifndef __GNUC__
#define FLETCH(SUM,BYTE) (SUM)->Checksum1 += (BYTE);                    \
                         if ((SUM)->Checksum1 > 254) (SUM)->Checksum1 -= 255; \
                         (SUM)->Checksum2 += (SUM)->Checksum1; \
                         if ((SUM)->Checksum2 > 254) (SUM)->Checksum2 -= 255;
#else
static inline void FLETCH(FLETCHER_CHECKSUM *checksums, uint8_t byte) {
    checksums->Checksum1 += byte;
    if (checksums->Checksum1 > 254) checksums->Checksum1 -= 255;
    checksums->Checksum2 += checksums->Checksum1;
    if (checksums->Checksum2 > 254) checksums->Checksum2 -= 255;
    return;
}
#endif



#ifdef __cplusplus
}
#endif


#endif                          //_USBCMDIO_INCLUDED
