/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT, but is a refactoring to move shell
    behavior into its own file (ie, out of main.c)

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    instr_task.c
 * @brief   instrument communications task handler, primarily USB cmds
 *
 * @{
 */

#include "ch.h"
#include "hal.h"
#include <strings.h>

#include   "bbi2c.h"
//#include "i2cdrv.h"

#include "chprintf.h"
#include "shell.h"
#include "bulk_usb.h"
#include "usbcmdio.h"

#include "instr_task.h"
#include "instr_debug.h"
#include "instr_error.h"   // error and status definitions: OK or HOK = 0, status is positive, errors negative
#include "buffer.h"
#include "OSandPlatform.h"
#include "instr_cmds.h"

usb_packet_t             pktInBuf;       // global instance of a   USB packet, max size = 254 bytes
usb_packet_t             dbgPktBuf;      // global debug USB packet
BulkUSBDriver            BDU1;           // global instance of the USB bulk   driver
extern uint_fast8_t      USBconfigured;  // global var    set when USB is configured
extern SerialUSBDriver   SDU1;           // global instance of the USB serial driver
extern const ShellConfig shell_cfg1;
extern buffer 		 cake;

#ifdef _TEST_BBI2C
static uint8_t  i2c_addr = 0x1c; //0b0011100; 
#endif // _TEST_BBI2C

#define PKTIO_TIMEOUT -1

// return:  number of bytes received... unless err
static int readPacket(usb_packet_t *buffer, systime_t tmo)
{
  uint_fast8_t pkt_size;
  uint_fast8_t nbytes;
  uint_fast8_t rval;
  systime_t tmoTime=chTimeNow() + tmo;
  
  //Wait for the first 4 bytes (header)
  do {
    rval=BDU1.vmt->readt(&BDU1,(uint8_t *)buffer,4,2);
    if (tmo && chTimeNow()>tmoTime) return(PKTIO_TIMEOUT);
  } while (rval<=0);
  pkt_size=*(uint8_t *)buffer;  //  first UCHAR of header
  nbytes=4;
  ORANGE_ON;
  
  do {
    //Timed read of up to pkt_size bytes, with 2-tick timeout
    rval=BDU1.vmt->readt(&BDU1,(uint8_t *)buffer+nbytes,pkt_size-nbytes,2);
    nbytes += rval;
  
    ORANGE_OFF;
    if (tmo && chTimeNow()>tmoTime) return(PKTIO_TIMEOUT);
  } while (nbytes<pkt_size);
  return(nbytes);
}

//  return:  number of bytes written ... unless err
//           Warning ... this will STALL if we fill up the virtual com port's
//           output and nobody's there to drain it off
static int writePacket(usb_packet_t *buffer, systime_t tmo)
{
  uint_fast8_t pkt_size;
  uint_fast8_t nbytes;
  uint_fast8_t rval;
  int nwritten=0;
  systime_t tmoTime=chTimeNow() + tmo;

  pkt_size=buffer->length;
  nbytes=pkt_size;
  do {
    rval=BDU1.vmt->writet(&BDU1,(uint8_t *)buffer,nbytes,2);//chibios command to write nbytes to BDU1
    nbytes -= rval;
    nwritten += rval;
    if (tmo && chTimeNow()>tmoTime) return(nwritten);
  } while(nbytes > 0);
  return(nwritten);
}


// Command Dispatcher Requirements:
// See usbcmdio.h for Mike's doc and implementation !!
//
//   Recognize 7 commands:  ACK, NAK, RESET, ID, WRITE, READ, ECHO

/*
 * This is the "instrument thread" 
 * Reads packets and dispatches to instrument control routines
 */

__attribute__((noreturn)) msg_t InstrumentThread(void *arg) {
  size_t rval; 
#ifndef RELEASE
  size_t wval;
#endif
//  uint16_t aCheckSum = 0;
  uint16_t aCheckSum = 2056; //Ryan checksum check
#ifdef _TEST_BBI2C
  uint8_t status;
  static uint8_t txbuf[32];
  static uint8_t rxbuf[32];
#endif // _TEST_BBI2C
  ChipDriverStatus_t chipStatus = SUCCESS;
  // volatile int32_t dly = 0, dmmy = 0;
  // int j,k;
  // uint8_t bit, pldata;
  
  (void)arg;
  chRegSetThreadName("Instrument");


  /* Reader thread loop.*/
  while (!USBconfigured) chThdSleep(100); //Wait here until USB hw is configured

  RED_OFF;
  while (TRUE) {
    bzero(&pktInBuf,sizeof(usb_packet_t));//Zeros out the mem location to store pktInBuf

    rval=readPacket(&pktInBuf,0);//Reads in a new packet
    
    // Command Dispatcher Requirements:
    // See usbcmdio.h for structures 
    //   defining each kind of message
    //   and payload
    //
    //   Recognize 9 commands:  ACK, NAK, RESET, ID, WRITE, READ, ECHO, SHADOW, ADC

    // DB1_HI;
    switch (pktInBuf.type) {
    case CMD_ACK:
      pktInBuf.length = 4;
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
#ifdef RELEASE
      writePacket(&pktInBuf,0);
#else
      wval=writePacket(&pktInBuf,0);
      dprintf("ACK sent %u \r\n",wval);
#endif
      break;
    case CMD_NAK:
      pktInBuf.length = 4;
      pktInBuf.type = CMD_ACK;        // all packet's ACK unless error
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
#ifdef RELEASE
      writePacket(&pktInBuf,0);
#else
      wval=writePacket(&pktInBuf,0);
      dprintf("NAK sent %u\r\n",wval);
#endif
      break;
    case CMD_RESET:

      pktInBuf.length = 4;
      if (chipStatus == SUCCESS) 
        pktInBuf.type = CMD_ACK;        // all packet's ACK unless error
      else
        pktInBuf.type = CMD_NAK; 
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
#ifdef RELEASE
      writePacket(&pktInBuf,0);
#else
      wval=writePacket(&pktInBuf,0);
      dprintf("RESET sent %u\r\n",wval);
#endif
      break;
    case CMD_ID:
      dprintf("ID \r\n");
      get_instrument_ID(&pktInBuf); // my_id;
      // pktInBuf.length=4+sizeof(payload_id_response_t);
      if (chipStatus == SUCCESS) 
        pktInBuf.type = CMD_ACK;        // all packet's ACK unless error
      else
        pktInBuf.type = CMD_NAK; 
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
#ifdef RELEASE
      writePacket(&pktInBuf,0);
#else
      wval=writePacket(&pktInBuf,0);
      dprintf("ID sent %u \r\n",wval);
#endif
      break;
    case CMD_ECHO:
      dprintf("ECHO \r\n");
      pktInBuf.type = CMD_ACK;        // all packet's ACK unless error
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
      if (rval > 0) {                 // echo the incoming packet, if non-zero
#ifdef RELEASE
        writePacket(&pktInBuf,0);
#else
        wval=writePacket(&pktInBuf,0);
        dprintf("ECHO sent %d\r\n",wval);
#endif
      }
      break;
    case CMD_SSN:
      dprintf("SSN \r\n");
      get_instrument_SSN(&pktInBuf);
      if (chipStatus == SUCCESS) 
        pktInBuf.type = CMD_ACK;
      else
        pktInBuf.type = CMD_NAK; 
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
      writePacket(&pktInBuf,0);
      break;
    case CMD_UID:
      dprintf("UID \r\n");
      get_instrument_UID(&pktInBuf);
      if (chipStatus == SUCCESS) 
        pktInBuf.type = CMD_ACK;
      else
        pktInBuf.type = CMD_NAK; 
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
      writePacket(&pktInBuf,0);
      break;
    case CMD_ADC:
      dprintf("ADC \r\n");
      palTogglePad(GPIOD, 12); //Debug check: Green LED
      get_instrument_ADC(&pktInBuf);
      if (chipStatus == SUCCESS) 
        pktInBuf.type = CMD_ACK;
      else
        pktInBuf.type = CMD_NAK; 
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
      writePacket(&pktInBuf,0);
      break;
    case CMD_ADC_ALL:
//    dprintf("ADC ALL \r\n");
      palTogglePad(GPIOD, 12); //Debug check: Green LED
      if(!cake.empty) {
//	      while(!cake.empty){ //seems to run into a race condition
	int k=0;
	      while(k<22){
	      	      get_instrument_ADC_ALL(&pktInBuf);
         	      pktInBuf.type = CMD_ACK;
		      pktInBuf.checksum = aCheckSum; 
		      writePacket(&pktInBuf,0);	
		      k++;
	      }
      } else {
		//TODO:fill a packet with zero and send it
		pktInBuf.type = CMD_NAK; 
		pktInBuf.length = 9;
		pktInBuf.checksum = aCheckSum;
		pktInBuf.payload.adc_resp.current =0;
		pktInBuf.payload.adc_resp.voltage =0;
		pktInBuf.payload.adc_resp.DUTstate =0;
		writePacket(&pktInBuf,0);	
      }
      break;
    case CMD_ADC_TEST2:
//    dprintf("ADC ALL \r\n");
      palTogglePad(GPIOD, 12); //Debug check: Green LED
      if(!cake.empty) {
      	      get_instrument_ADC_TEST(&pktInBuf);//Put 2 readings in a packet
	      if (chipStatus == SUCCESS) 
		pktInBuf.type = CMD_ACK;
	      else
		pktInBuf.type = CMD_NAK; 
	        pktInBuf.checksum = aCheckSum;
	        writePacket(&pktInBuf,0);	
      } else {
	pktInBuf.length = 4;
      pktInBuf.type = CMD_NAK;        // packet's NAK on empty
      pktInBuf.checksum = aCheckSum;  
      writePacket(&pktInBuf,0);		//send empty packet NAK

      }
      break;
    default:
      dprintf("ERROR: unrecognized command: %u\r\n",pktInBuf.type);
      pktInBuf.length = 4;
      pktInBuf.type = CMD_NAK;        // packet's NAK on error
      // aCheckSum = compute_fletch(&pktInBuf);
      pktInBuf.checksum = aCheckSum;  // TODO: compute FLETCH
      writePacket(&pktInBuf,0);
      break;
    }

#ifdef _SPI_TEST
    spiSelect(&SPID2);
    spiSend(&SPID2, 8, txbuf);
    spiUnselect(&SPID2);
#endif // _SPI_TEST

#ifdef _TEST_BBI2C
#ifdef _BBI2C_INCLUDED
    bzero(txbuf,32);
    bzero(rxbuf,32);
    status = bbI2C_bufio(i2c_addr<<1, 0x80, txbuf, 0, rxbuf, 1);
    dprintf("Global reg 0x%x     status: %d\r\n",rxbuf[0],status);
    status = bbI2C_bufio(i2c_addr<<1, 0x0, txbuf, 12, rxbuf, 0);
    dprintf("Clear reg A chan 0 status: %d\r\n",status);
    status = bbI2C_bufio(i2c_addr<<1, 0x20, txbuf, 12, rxbuf, 0);
    dprintf("Clear reg B chan 0 status: %d\r\n",status);
    status = bbI2C_bufio(i2c_addr<<1, 0x40, txbuf, 12, rxbuf, 0);
    dprintf("Clear reg A chan 1 status: %d\r\n",status);
    status = bbI2C_bufio(i2c_addr<<1, 0x60, txbuf, 12, rxbuf, 0);
    dprintf("Clear reg B chan 1 status: %d\r\n",status);
#endif // _BBI2C_INCLUDED
#endif // _TEST_BBI2C

    //Blink the LED
    RED_ON;
    
    //Here's how to printf out the USB debug shell port
    //chprintf(shell_cfg1.sc_channel,".");
  }
}

