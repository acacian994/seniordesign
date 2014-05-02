/*******************************************************************************
*           Copyright (C) 2013 Tektronix Inc., All rights reserved.
*
*                       3841 Brickway Blvd. Suite 210
*                       Santa Rosa, CA 95403
*                       Tel:(707) 595-4770
*
* Filename:     instr_cmds.c
*
* Description:  LE320 commands, typically called from the dispatcher, instr_task
*
* $Author$
* $DateTime$
* $Id$
*******************************************************************************/
//#define TRACE_PRINT 1

#include "OSandPlatform.h"
#define GLOBAL_VERSION      // this prevents "extern" prefix on version.h symbols
#include "version.h"        // Git SHA1 and changeset

#define GLOBAL_INSTR_CMDS   // this manages "extern" prefix on this file's symbols

// #include "hmc6545.h"         // low-level Hittite chip drivers
#include "ch.h"              // for base classes used by chprintf.h
#include "chprintf.h"        // for access to task-aware printf's
#include "shell.h"           // for access to task-aware debug cmds
#include "bulk_usb.h"
#include "instr_cmds.h"
#include "md5_tek.h"
#include "buffer.h"

extern usb_packet_t      pktInBuf;       // global instance of a   USB packet, max size = 254 bytes
extern usb_packet_t      dbgPktBuf;
extern buffer 		 cake;
extern readings 	 cur_reading;

// NOTE: most of the ID is still hard-coded
// get fw id:  interrogate version.h structure for ID info
//             this function is hard coded to:
//             major.minor.build =  00.01.<build count>
// 5/15/2013   report the SHA1 of the Git firmware repo from which built
//
// The SHA1 is an auto-build value scripted in version.mk
// It is part of the changeset array, which consists of build number,
// followed by an underscore, then 7 digits of SHA1, or + and six digits
// of the SHA1 if the repo was built with any changed file (dirty repo)
#define SZ_BUILD_CNT_ATOI 4      // size of scratch-pad for atoi call

// #define SZ_BUILD_SHA1     7   // version.mk now generates this !!
void get_instrument_ID(usb_packet_t *buffer) {
  payload_id_response_t myID;
  char *pSha1    = NULL;
  char *pBldInfo = NULL;
  char *pChars   = NULL;
  char bldStr[5];    // scratch-pad for atoi: converts build-num to integer
  char bldInfo[200]; // the other build-info
  int k;

  memset(&myID,   0,sizeof(myID));
  memset(&bldStr, 0,sizeof(bldStr));  // NULL atoi conversion array
  memset(&bldInfo,0,sizeof(bldInfo)); // NULL array
  myID.productID       = 1;      // start at 1, max 255
  myID.protocolVersion = 1;      // increment when the ID pkt format changes
  myID.fwRev_major     = 1;      // range: 0-99
  myID.fwRev_minor     = 15;     // range: 0-99

  // copy the version.h build number (changeset) to a small array
  //      for conversion to an integer (atoi)
  pSha1  = (char *)&build_sha1[0];    // first 4 chars: 0092_+ae431f
  pChars = (char *)&bldStr[0];        // point to scratch-pad for atoi
  for (k=0;k<SZ_BUILD_CNT_ATOI;k++) { // copy build-count from changeset
    *pChars++ = *pSha1++;
  }
  myID.fwRev_build = (uint16_t) atoi(&bldStr[0]);
  pSha1++;  // move past underscore

  // copy the SHA1 from the changeset
  pChars = (char *)&myID.bld_sha[0];
  for (k=0;k<SZ_BUILD_SHA1;k++) {         // copy SHA1 from changeset
	*pChars++ = *pSha1++;
  }
  *pChars++ = 0x00;  // null-terminate
  myID.bld_sha_len = SZ_BUILD_SHA1+1;  // null-terminate-len

  pBldInfo = (char *)&build_info[0];
  pChars   = (char *)&myID.bld_info[0];
  for (k=0;k<(int)sizeof(build_info);k++) {         // copy SHA1 from changeset
	*pChars++ = *pBldInfo++;
  }
  *pChars++ = 0x00;  // null-terminate
  myID.bld_info_len = sizeof(build_info) + 1;

  // copy myID into the payload
  buffer->payload.id_resp = (payload_id_response_t)myID;
  buffer->length = 4 + 16 + myID.bld_info_len; 
} // end get_instrument_ID


void print_ID(BaseSequentialStream *chp, usb_packet_t *pPkt) {
  payload_id_response_t myID = (payload_id_response_t)pPkt->payload.id_resp;
  char *pStr = (char*) &(myID.bld_sha[0]);

  chprintf(chp, "LE320 ID \r\n");
  chprintf(chp, "  FW REV   %d.%d.%d \r\n",
           myID.fwRev_major,  myID.fwRev_minor, myID.fwRev_build);
  chprintf(chp, "  SHA1     %s \r\n", pStr);
} // end print_ID


void print_SSN(BaseSequentialStream *chp, usb_packet_t *pPkt) {
  payload_ssn_t mySSN = (payload_ssn_t)pPkt->payload.ssn_resp;

  chprintf(chp, "  SSN      0x%.8X%.8X%.8X \r\n",
           mySSN.ssn_values[2], 
           mySSN.ssn_values[1], 
           mySSN.ssn_values[0]);
} // end print_SSN


void print_UID48(BaseSequentialStream *chp, MD5_TEK *pHash) {
  // payload_ssn_t mySSN = (payload_ssn_t)pPkt->payload.ssn_resp;
  int k=0;

  chprintf(chp, "  UID-128  0x");
  for (k = 0; k < 16; k++) {
	  chprintf(chp, "%.2x", pHash->digest[k]);
  }
  chprintf(chp, "\r\n");

  chprintf(chp, "  UID-48   0x");
  for (k = 0; k < 6; k++) {
	  chprintf(chp, "%.2x", pHash->digest[k]);
  }
  chprintf(chp, "\r\n");
} // end print_UID48


static uint32_t *pSSN_ENTRY = (uint32_t*)0x1fff7a10;
static uint8_t SSN_AS_BYTES_SZ = 12;

// read the chip silicon serial number (SSN)
// for STM32F4, it is a 96-bit Unique Device ID (UID)
//     which can be read in three 32-bit chunks
// the LPC43XX has a 128-bit SSN, which requires
//     the IAP ROM jump-code to run and return (see below)
void get_instrument_SSN(usb_packet_t *buffer) {
  payload_ssn_t mySSN;
  int k = 0;

  memset(&mySSN,0,sizeof(mySSN));
  for (k=0;k<3;k++) {
	mySSN.ssn_values[k] = *(pSSN_ENTRY+k);  // little-endian, low-order first
  }
  mySSN.ssn_cnt = 3;
  buffer->payload.ssn_resp = (payload_ssn_t)mySSN;
  buffer->length = 4 + 1 + 15; // 1=ssn_cnt, 12 = (4 bytes per ssn_value) * 3 + 3 dummy pads
} // end get_instrument_SSN


// ssn_to_MD5:  this is the STM32 version, assumes a 96-bit SSN
MD5_TEK ssn_to_MD5(void) {
  MD5_TEK  myHash;
  uint32_t ssn_uint32[3];
  uint8_t  ssn_uint8[SSN_AS_BYTES_SZ]; // digest input data
  uint8_t *pbDD  = NULL;               // pointer to byte, digest data
  uint8_t *pbSSN = NULL;               // pointer to byte, values of mySSN
  int j,k;

  // Read the chip's unique ID (silicon serial number -> SSN)
  for (k=0;k<3;k++) {
	ssn_uint32[k] = *(pSSN_ENTRY+k);  // little-endian, low-order first
  }

  // copy SSN into md5  context buffer, a byte at a time
  pbDD  = &ssn_uint8[0];
  for (k=0;k<3;k++) {     // 96-bits: operate on 3 32-bit chunks
	pbSSN = (uint8_t*)&(ssn_uint32[k]);
	for (j=0;j<4;j++) {   // copy 4 bytes of SSN to digest
	  *pbDD++ = *pbSSN++;
	}
  }

  MD5Init  (&myHash);
  MD5Update(&myHash, (unsigned char*)&ssn_uint8, SSN_AS_BYTES_SZ);
  MD5Final (&myHash);
  return     myHash;
} // end ssn_to_MD5

// compute the chip UID, a 3-step operation
//     1  read the chip silicon serial number (SSN)
//     2  digest the SSN using MD5 hash
//     3  load payload with the first 48-bits of MD5
void get_instrument_UID(usb_packet_t *buffer) {
  payload_uid_t myUID;
  MD5_TEK  myHash;
  uint32_t ssn_uint32[3];
  uint8_t  ssn_uint8[SSN_AS_BYTES_SZ]; // digest input data
  uint8_t *pbDD  = NULL;               // pointer to byte, digest data
  uint8_t *pbSSN = NULL;               // pointer to byte, values of mySSN
  int j,k;
  memset(&myUID,0,sizeof(myUID));

  // Read the chip's unique ID (silicon serial number -> SSN)
  for (k=0;k<3;k++) {
	ssn_uint32[k] = *(pSSN_ENTRY+k);  // little-endian, low-order first
  }

  // copy SSN into md5 context buffer, a byte at a time
  pbDD  = &ssn_uint8[0];
  for (k=0;k<3;k++) {     // 96-bits: operate on 3 32-bit chunks
	pbSSN = (uint8_t*)&(ssn_uint32[k]);
	for (j=0;j<4;j++) {   // copy 4 bytes of SSN to digest
	  *pbDD++ = *pbSSN++;
	}
  }

  MD5Init  (&myHash);
  MD5Update(&myHash, (unsigned char*)&ssn_uint8, SSN_AS_BYTES_SZ);
  MD5Final (&myHash);

  for (k=0;k<6;k++) {  // copy first 48 bits of hash (6 bytes) to payload
	  myUID.uid_values[k] = myHash.digest[k];
  }

  buffer->payload.uid_resp = (payload_uid_t)myUID;
  buffer->length = 4 + 6; // 1=uid_cnt, 6 = (6 bytes of UID)
} // end get_instrument_UID

void get_instrument_ADC(usb_packet_t *buffer){
	payload_adc_t myADC;
	readings next_reading;
	memset(&myADC,0,sizeof(myADC));
#if 0
	if(!cake.empty){ //if there is a reading waiting snag it and put it in the payload
		next_reading=readbuffer(&cake);
		myADC.current= next_reading.current;
		myADC.voltage= next_reading.voltage;
		myADC.DUTstate= next_reading.DUTstate;
	}/*else{ //if not just grab what is on the spi interface (mostly test case)
		myADC.voltage= cur_reading.voltage;
		myADC.current= cur_reading.current;
		myADC.DUTstate= cur_reading.DUTstate;
	}*/
#endif
#if 1
	next_reading=readbuffer(&cake);//Buffer automaticly returns zero on empty
	myADC.current= next_reading.current;
	myADC.voltage= next_reading.voltage;
	myADC.DUTstate= next_reading.DUTstate;
#endif
#if 0
	myADC.voltage= 500;
	myADC.current= 40;
	myADC.DUTstate= 99;
#endif
	buffer->payload.adc_resp = (payload_adc_t)myADC;
//	buffer->length = 4 + 5 + (myADC.numReadings*8); //dynamic length setting
	buffer->length = 4 + 5; //1=cmd_usb, 2 voltage, 2 current, 1 DUT
}

void get_instrument_ADC_ALL(usb_packet_t *buffer){
	payload_adc_all_t myADC_ALL;
	readings next_reading; //packet can hold 24 readings
			       //EP3 maxpacket size = 0x0040
	uint8_t numReadings =0;
	int i = 0;
	memset(&myADC_ALL,0,sizeof(myADC_ALL));
	while(!cake.empty && i < 24) {
		next_reading=readbuffer(&cake);
		myADC_ALL.values[i].current= next_reading.current;
		myADC_ALL.values[i].voltage= next_reading.voltage;
		myADC_ALL.values[i].DUTstate= next_reading.DUTstate;
		i++;
		numReadings = i;
	}
	/*
	for (i=0;i<=39;i++){ //take 40 readings from buffer
		if(!cake.empty){ //if there is a reading waiting snag it and put it in the payload (empty=1 when empty)
			next_reading=readbuffer(&cake);
			myADC_ALL.values[i].current= next_reading.current;
			myADC_ALL.values[i].voltage= next_reading.voltage;
			myADC_ALL.values[i].DUTstate= next_reading.DUTstate;
			numReadings = i;
		}
	}
	*/
	buffer->payload.adc_all_resp = (payload_adc_all_t)myADC_ALL;
	buffer->length = 4 + (numReadings*5); //dynamic length setting,1=cmd_usb, (2 voltage, 2 current, 1 DUT)*5
//	buffer->length = 4 + 6; //1=cmd_usb, 2 voltage, 2 current, 1 DUT
}

void get_instrument_ADC_TEST(usb_packet_t *buffer){
	payload_adc_all_t myADC_ALL;
	readings next_reading; //packet can hold 24 readings
			       //EP3 maxpacket size = 0x0040
	uint8_t numReadings =0;
	int i = 0;
	memset(&myADC_ALL,0,sizeof(myADC_ALL));
	while(!cake.empty && i < 2) {
		next_reading=readbuffer(&cake);
		myADC_ALL.values[i].current= next_reading.current;
		myADC_ALL.values[i].voltage= next_reading.voltage;
		myADC_ALL.values[i].DUTstate= next_reading.DUTstate;
		i++;
		numReadings = i;
	}

	buffer->payload.adc_all_resp = (payload_adc_all_t)myADC_ALL;
	buffer->length = 4 + (numReadings*5); //dynamic length setting,1=cmd_usb, (2 voltage, 2 current, 1 DUT)*5

}

void print_ADC(BaseSequentialStream *chp, usb_packet_t *pPkt) {
	payload_adc_t myADC = (payload_adc_t)pPkt->payload.adc_resp;

	chprintf(chp, "%d.%d.%d \r\n",myADC.current,myADC.voltage,myADC.DUTstate);
}

// The following code 
typedef struct
{
  unsigned int ReturnCode;
  unsigned int Result[4];
} IAP_return_TypeDef;
 
IAP_return_TypeDef iap_return;
 
// NXP LE43xx IAP ROM runs code to return 128-bit UID
#define NXP_IAP_ADDRESS 0x1FFF1FF1
unsigned NXP_iap_param_table[5];
 
void iap_entry(unsigned param_tab[],unsigned result_tab[])
{
  void (*iap)(unsigned [],unsigned []);iap = (void (*)(unsigned [],unsigned []))NXP_IAP_ADDRESS;
  iap(param_tab,result_tab);
}
