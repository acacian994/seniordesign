/*
  ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
  2011,2012 Giovanni Di Sirio.

  This file is part of ChibiOS/RT.

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

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "shell.h"

#include "cmd_shell.h"

#include "bulk_usb.h"
#include "usbcfg.h"
#include <strings.h>

#include "instr_task.h"
#include "instr_debug.h"       // common debug macros:  dprintf
//R
#include "stm32f4xx_exti.h"	//External Interupt Structs
#include "stm32f4xx_syscfg.h"	//Sysconfig to enable External Interupt Structs
#include "misc.h"		//setting the NVIC IRQ channel mapped to the EXTI we want
#include "stm32f4xx_spi.h"	//SPI Structs
#include "stm32f4xx_rcc.h"	//Perf clock Structs
#include "buffer.h"		//Circle buffer
#define ADCv1 0
#define ADCv2 1
#define HOST 1
#define DUT 0

extern SerialUSBDriver SDU1;   // virtual serial port over USB
extern BulkUSBDriver BDU1;
extern const ShellConfig shell_cfg1;

//Prototypes
uint16_t mySPI_GetData(uint16_t);
//static void extcb15(EXTDriver *, expchannel_t);
//static void extcb2(EXTDriver *, expchannel_t);
static void extcb15(void);
static void extcb2(void);
static uint8_t DUTstatus = 0x05; //each bit will be the status of the GPIO pins.
static uint16_t rxbuf[512]; //inital SPIrx buffer
static uint16_t txbuf[512]; //inital SPItx buffer
buffer cake; //Circle buffer for storage, adjust its max size in buffer.h
readings cur_reading; // Reading from the SPI and DUT pins
readings send_reading; // Readings to send in a packet
uint16_t i=0;


/*
 * SPI2 configuration structure.
 * Speed 21MHz, CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
 * The slave select line is the pin 12 on the port GPIOB.
 */

static const SPIConfig spi2cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOB,
  12,
  0
};
#if 0
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((uint16_t)0x0001)            /*!<Clock Phase */
#define  SPI_CR1_CPOL                        ((uint16_t)0x0002)            /*!<Clock Polarity */
#define  SPI_CR1_MSTR                        ((uint16_t)0x0004)            /*!<Master Selection */

#define  SPI_CR1_BR                          ((uint16_t)0x0038)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((uint16_t)0x0008)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        ((uint16_t)0x0010)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        ((uint16_t)0x0020)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         ((uint16_t)0x0040)            /*!<SPI Enable */
#define  SPI_CR1_LSBFIRST                    ((uint16_t)0x0080)            /*!<Frame Format */
#define  SPI_CR1_SSI                         ((uint16_t)0x0100)            /*!<Internal slave select */
#define  SPI_CR1_SSM                         ((uint16_t)0x0200)            /*!<Software slave management */
#define  SPI_CR1_RXONLY                      ((uint16_t)0x0400)            /*!<Receive only */
#define  SPI_CR1_DFF                         ((uint16_t)0x0800)            /*!<Data Frame Format */
#define  SPI_CR1_CRCNEXT                     ((uint16_t)0x1000)            /*!<Transmit CRC next */
#define  SPI_CR1_CRCEN                       ((uint16_t)0x2000)            /*!<Hardware CRC calculation enable */
#define  SPI_CR1_BIDIOE                      ((uint16_t)0x4000)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((uint16_t)0x8000)            /*!<Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((uint8_t)0x01)               /*!<Rx Buffer DMA Enable */
#define  SPI_CR2_TXDMAEN                     ((uint8_t)0x02)               /*!<Tx Buffer DMA Enable */
#define  SPI_CR2_SSOE                        ((uint8_t)0x04)               /*!<SS Output Enable */
#define  SPI_CR2_ERRIE                       ((uint8_t)0x20)               /*!<Error Interrupt Enable */
#define  SPI_CR2_RXNEIE                      ((uint8_t)0x40)               /*!<RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((uint8_t)0x80)               /*!<Tx buffer Empty Interrupt Enable */
/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((uint8_t)0x01)               /*!<Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((uint8_t)0x02)               /*!<Transmit buffer Empty */
#define  SPI_SR_CHSIDE                       ((uint8_t)0x04)               /*!<Channel side */
#define  SPI_SR_UDR                          ((uint8_t)0x08)               /*!<Underrun flag */
#define  SPI_SR_CRCERR                       ((uint8_t)0x10)               /*!<CRC Error flag */
#define  SPI_SR_MODF                         ((uint8_t)0x20)               /*!<Mode fault */
#define  SPI_SR_OVR                          ((uint8_t)0x40)               /*!<Overrun flag */
#define  SPI_SR_BSY                          ((uint8_t)0x80)               /*!<Busy flag */
#endif
/*Ryan added 
 * SPI1 configuration structure.this is the setup for SPIv1
 * I think stm32f4xx says we use SPIv2, wich has a 2nd control register
 * each CR is 16 bits wide
 * Speed 21MHz, CPHA=0, CPOL=0, 16bits frames, MSb transmitted first.
 * OR in options: these go into the CR1 register
 * SPI_CR1_CPHA to change phase, SPI_CR1_CPOL to change polarity, SPI_CR1_DFF for 16 bit frames
 * SPI_CR1_MSTR for master select, SPI_CR1_SPE for SPI enable
 * SPI_CR1_BIDIOE for Output enable in bidirectional mode, SPI_CR1_BIDIMODE for Bidirectional data mode enable
 * SPI_CR1_RXONLY for Recieve only
 * SPI_CR1_BR_0 though 2 for baud rate scaler
 * | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2
 * The slave select line is the pin 4 on the port GPIOA.
 */
static const SPIConfig spi1cfg = {
  /* Callback function.*/
  NULL,
  /* HW dependent part.*/
  GPIOA,
  4,
  SPI_CR1_CPHA | SPI_CR1_DFF | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_BR_0
};

/* Ryan added
 * SPI end transfer callback.
 */
static void spirxcb(SPIDriver *spip) {
/*for ADC read this might want to be where we poll for DUT flags and send them to the circle buffer */
  /* On transfer end just releases the slave select line.*/
//  chSysLockFromIsr();
//  spiUnselectI(spip);
//  chSysUnlockFromIsr();
}

/* Ryan added Freq and cycle are randomly picked atm, set for very slow
 * PWM.setup
 */
static const PWMConfig pwm2cfg = {
  20000,                                  /* 10kHz PWM clock frequency. , was set to 5000 */
  2,                                  /* PWM period is 10000 cycles. Should be 1 second. Total of on+off , was set to 1000 */
  NULL,					/*callback pointer, this is the other option to start a dut flag capture here*/
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},	/*Channel 3 enabled*/
   {PWM_OUTPUT_DISABLED, NULL}
  },
  /* HW dependent part.*/
  0
};
void myCHSPI_Init(void){

  spiStart(&SPID1, &spi1cfg);
  palSetPad(GPIOA, 4);
  palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_HIGHEST);   /* NSS.     */
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);	/* SCK.     */
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);	/* MISO.    */
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);	/* MOSI.    */
}
void mySTSPI_Init(void){
  SPI_InitTypeDef SPI1_Init;
//  SPI_StructInit(&SPI1_Init);
//  SPIx_CLK_INIT(SPIx_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
   /* Ryan added 
    * Configures SPI 1 to be 
    * Master
    * Full Duplex (so SCK can be started/stopped)
    * 16 bit datasize
    * MSB recieved first
    * NSS controled by hardware (GPIO setting)
    * data is valid on 2 edges as per the ADC datasheet.
    * BaudRatePrescaler = 16 , options are 2,4,8,16,32,64,128,256
    * Clock Polarity = Low
    */
  SPI1_Init.SPI_Mode = SPI_Mode_Master;
  SPI1_Init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI1_Init.SPI_DataSize = SPI_DataSize_16b;
  SPI1_Init.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI1_Init.SPI_NSS = SPI_NSS_Hard;
  SPI1_Init.SPI_CPHA = SPI_CPHA_1Edge;
  SPI1_Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
  SPI1_Init.SPI_CPOL = SPI_CPOL_Low;

//  SPI_DataSizeConfig(SPI1, SPI_DataSize_16b);
  SPI_Init(SPI1, &SPI1_Init);
  SPI_Cmd(SPI1, ENABLE); //start SPI1 as per STM32F4xx_spi.h

   /* Ryan added
   * Initializes the SPI driver 1. The SPI1 signals are routed as follow:
   * PA4 - NSS.
   * PA5 - SCK.
   * PA6 - MISO.
   * PA7 - MOSI.
   */
  palSetPad(GPIOA, 4);
  palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_HIGHEST);/* NSS.     */
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);	/* SCK.     */
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);	/* MISO.    */
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);	/* MOSI.    */


}
void myGPIO_Init(void){
 /*	Ryan GPIO SETUP
  *	PalSetPad sets pin to logical HI
  *	PalClearPad sets pin to logical LOW
  *	ADC Pins:
  */
/****************** Inital GPIO pinout for ADCv1 *************************/	
#if ADCv1
  palSetPadMode(GPIOE,0, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL);// out-> EXT/INT *NC
  palSetPad(GPIOE, 0);
  palSetPadMode(GPIOE,1, PAL_MODE_INPUT);// in <- SYNC  *NC
  palSetPadMode(GPIOE,7, PAL_MODE_OUTPUT_PUSHPULL);// out->DIVSCLK0 *PD
  palClearPad(GPIOE, 7);
  palSetPadMode(GPIOE,8, PAL_MODE_OUTPUT_PUSHPULL);// out->DIVSCLK1
  palClearPad(GPIOE, 8);
  palSetPadMode(GPIOE,9, PAL_MODE_OUTPUT_PUSHPULL);// out->INVSYNC
  palClearPad(GPIOE, 9);
  palSetPadMode(GPIOE,12, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL);// out->RD
  palClearPad(GPIOE, 12);
  palSetPadMode(GPIOE,13, PAL_MODE_INPUT);// in <- RDERR
  palSetPadMode(GPIOE,14, PAL_MODE_INPUT);// in <- EOC 
  palSetPadMode(GPIOE,15, PAL_MODE_INPUT|PAL_STM32_PUDR_FLOATING);// in <- BUSY, has exti tied to it to watch for low
//   * PA4 - NSS.
//   * PA5 - SCK.
//   * PA6 - MISO.
//   * PA7 - MOSI.
/*************Hardwired Ports on ADC****************/
//   * SYNC - FLOAT
//   * EXT/INT - HI
//   * RD - LOW
//   * CS -LOW
//   * A/B - FLOAT
//   * Impulse - LOW
//   * A0 - LOW
//   * Byteswap - FLOAT
//   * InvSCLk - FLOAT
//   * 
#endif

/**************************** GPIO pinout for ADCv2 ***************************************/
#if ADCv2
//palSetPadMode(GPIOE,0, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL);// out-> EXT/INT *NC
//palSetPad(GPIOE, 0); High for STmicro lock
//palSetPadMode(GPIOE,1, PAL_MODE_INPUT);// in <- SYNC  *NC
  palSetPadMode(GPIOE,7, PAL_MODE_OUTPUT_PUSHPULL);// out->*PD
  palClearPad(GPIOE, 7); //High to Power down
  palSetPadMode(GPIOD,13, PAL_MODE_OUTPUT_PUSHPULL);// out->DIVSCLK0
  palClearPad(GPIOD, 13);
  palSetPadMode(GPIOB,15, PAL_MODE_OUTPUT_PUSHPULL);// out->DIVSCLK1
  palClearPad(GPIOB, 15);
  palSetPadMode(GPIOB,10, PAL_MODE_OUTPUT_PUSHPULL);// out->INVSYNC *Not sure of correct setting
  palSetPad(GPIOB, 10);
//palSetPadMode(GPIOE,12, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL);// out->RD *NC
//palClearPad(GPIOE, 12);
  palSetPadMode(GPIOE,13, PAL_MODE_INPUT);// in <- RDERR
  palSetPadMode(GPIOE,10, PAL_MODE_INPUT);// in <- EOC 
  palSetPadMode(GPIOE,12, PAL_MODE_INPUT|PAL_STM32_PUDR_FLOATING);// in <- BUSY, has exti tied to it to watch for low #adjust EXTI#
  palSetPadMode(GPIOE,15, PAL_MODE_OUTPUT_PUSHPULL); // out-> SDIN //Low for current data 
  palClearPad(GPIOE, 15);
  palSetPadMode(GPIOE,9, PAL_MODE_OUTPUT_PUSHPULL);// out->RESET *
  palClearPad(GPIOE, 9); //High to Reset
//   * PA4 - NSS.
//   * PA5 - SCK.
//   * PA6 - MISO.
//   * PA7 - MOSI
//   * PA2 - PWM2 ->CNVST 
/*************Hardwired Ports on ADC****************/
//   * SYNC - 
//   * EXT/INT - 
//   * RD - 
//   * CS -
//   * A/B - 
//   * Impulse - 
//   * A0 - 
//   * Byteswap - 
//   * InvSCLk - 
//   * 
//   * 

#endif	

	/* DUT input GPIO Pins (Can be SPI3) */	
#if HOST
  palSetPadMode(GPIOA,15, PAL_MODE_INPUT);// SPI3_NSS / DUT 1 MSB
  palSetPadMode(GPIOC,10, PAL_MODE_INPUT);// SPI3_SCK / DUT 2
  palSetPadMode(GPIOC,11, PAL_MODE_INPUT);// SPI3_MISO / DUT 3
  palSetPadMode(GPIOC,12, PAL_MODE_INPUT);// SPI3_MOSI / DUT 4
	/* DUT input GPIO Pins */
  palSetPadMode(GPIOD,2, PAL_MODE_INPUT);// DUT 5
  palSetPadMode(GPIOD,1, PAL_MODE_INPUT);// DUT 6
  palSetPadMode(GPIOD,3, PAL_MODE_INPUT);// DUT 7
  palSetPadMode(GPIOD,6, PAL_MODE_INPUT);// DUT 8 ##is also UART2 so change to D4 LSB
#endif

#if DUT
/***********Test setup for DUT Poll************/
  palSetPadMode(GPIOD,1, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);//
  palSetPadMode(GPIOD,2, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);//
  palSetPadMode(GPIOD,3, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);//
  palSetPadMode(GPIOD,6, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);//
	/* DUT input GPIO Pins (Can be SPI3) */	
  palSetPadMode(GPIOC,10, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);// SPI3_SCK
  palSetPadMode(GPIOC,11, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);// SPI3_MISO
  palSetPadMode(GPIOC,12, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);// SPI3_MOSI
  palSetPadMode(GPIOA,15, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OTYPE_PUSHPULL|PAL_STM32_PUDR_FLOATING);// SPI3_NSS
#endif

		//Temp sensor pins
  palSetPadMode(GPIOE,8, PAL_MODE_OUTPUT_PUSHPULL);// out-> Temp CS
  palSetPad(GPIOE, 8); 
}
void myCHEXTI_Init(void){
	extInit();
/**********************EXTI config for ADCv1**********************************/
#if ADCv1
	static const EXTConfig extcfg = {
	  {
	    {EXT_CH_MODE_DISABLED, NULL},//0
	    {EXT_CH_MODE_DISABLED, NULL},//1
	    {EXT_CH_MODE_DISABLED, NULL},//2
//	    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb2},//2 just setup as an LED blink to conferm its working
	    {EXT_CH_MODE_DISABLED, NULL},//3
	    {EXT_CH_MODE_DISABLED, NULL},//4
	    {EXT_CH_MODE_DISABLED, NULL},//5
	    {EXT_CH_MODE_DISABLED, NULL},//6
	    {EXT_CH_MODE_DISABLED, NULL},//7
	    {EXT_CH_MODE_DISABLED, NULL},//8
	    {EXT_CH_MODE_DISABLED, NULL},//9
	    {EXT_CH_MODE_DISABLED, NULL},//10
	    {EXT_CH_MODE_DISABLED, NULL},//11
	    {EXT_CH_MODE_DISABLED, NULL},//12
	    {EXT_CH_MODE_DISABLED, NULL},//13
	    {EXT_CH_MODE_DISABLED, NULL},//14
	    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, extcb15},//15 Busy pin from ADC
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL}
	  }
	};
#endif
/**********************EXTI config for ADCv2**********************************/
#if ADCv2
	static const EXTConfig extcfg = {
	  {
	    {EXT_CH_MODE_DISABLED, NULL},//0
	    {EXT_CH_MODE_DISABLED, NULL},//1
	    {EXT_CH_MODE_DISABLED, NULL},//2
//	    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb2},//2 just setup as an LED blink to conferm its working
	    {EXT_CH_MODE_DISABLED, NULL},//3
	    {EXT_CH_MODE_DISABLED, NULL},//4
	    {EXT_CH_MODE_DISABLED, NULL},//5
	    {EXT_CH_MODE_DISABLED, NULL},//6
	    {EXT_CH_MODE_DISABLED, NULL},//7
	    {EXT_CH_MODE_DISABLED, NULL},//8
	    {EXT_CH_MODE_DISABLED, NULL},//9
	    {EXT_CH_MODE_DISABLED, NULL},//10
	    {EXT_CH_MODE_DISABLED, NULL},//11
	    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, extcb15},//12 Busy pin from ADC
	    {EXT_CH_MODE_DISABLED, NULL},//13
	    {EXT_CH_MODE_DISABLED, NULL},//14
	    {EXT_CH_MODE_DISABLED, NULL},//15
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL},
	    {EXT_CH_MODE_DISABLED, NULL}
	  }
	};
#endif
	extStart(&EXTD1, &extcfg);
}
void mySTEXTI_Init(void){
 /*	In order to use an I/O pin as an external interrupt source, follow
  *          steps below:
  *            1- Configure the I/O in input mode using GPIO_Init()
  *            2- Select the input source pin for the EXTI line using SYSCFG_EXTILineConfig()
  *            3- Select the mode(interrupt, event) and configure the trigger 
  *               selection (Rising, falling or both) using EXTI_Init()
  *            4- Configure NVIC IRQ channel mapped to the EXTI line using NVIC_Init()
  */
	EXTI_InitTypeDef EXTI_InitStructure;
//	EXTI_StructInit(&EXTI_InitStructure); //might not need to call this, basically the inverse of below
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);
	/* external interupt initialization */
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //double check that this is the correct channel
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}
uint16_t SPI1_recv(void){
/****Tested, Hangs with Chip select low, SCK never moves***********/
	uint16_t temp = 0;
	spiAcquireBus(&SPID1); //for multithreaded SPI locking
	SPI1->DR = 0xAAAA; //dummy load into data register to start clock
	/* stm32f4xx.h defines SPI_SR_RXNE as the flag for recive not empty */
	while(!(SPI1->SR & SPI_SR_RXNE));//wait untill recive complete SPI_I2S_FLAG_RXNE 
//	while(!(SPI1->SR)) /* SR is the Status Register */
	temp = SPI1->DR; // DR is the SPI Data Register
	/* stm32f4xx.h defines SPI_SR_BSY as the spi1-sr flag for busy*/
	while(SPI1->SR & SPI_SR_BSY); //Wait untill SPI is not busy anymore SPI_I2S_FLAG_BSY
	spiReleaseBus(&SPID1); //for multithreaded SPI locking
	return temp;
}

void mychSPI_recieve(void){
/**************Tested, Hangs with chip select low, SCK never moves***********/
    spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
    palSetPad(GPIOD, GPIOD_LED5);       /* LED ON.                          */
    spiStart(&SPID1, &spi1cfg);         /* Setup transfer parameters.       */
    spiSelect(&SPID1);                  /* Slave Select assertion.          */
    spiExchange(&SPID1, 2, txbuf, rxbuf);/* Atomic transfer operations.The #is number of transactions total*/
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
    palClearPad(GPIOD, GPIOD_LED5);
}

uint16_t mySPI_GetData(uint16_t address){
/* Tested, hangs with chip select low, SCK never moves */
//Ported in from an example that reads MEM chip data wich is hooked to spi1. So some settings
//may be unneeded.
	palClearPad(GPIOA, 4);  //chip select low
	ORANGE_OFF;
	BLUE_OFF;
	RED_OFF;
	GREEN_OFF;
	ORANGE_ON;
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI_I2S_SendData(SPI1,address);
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI_I2S_ReceiveData(SPI1);  //Clear RXNE Bit

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI_I2S_SendData(SPI1,0x00); //dummy byte to generate clock
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));

	palSetPad(GPIOA, 4); //chip select hi
	ORANGE_OFF;
	return SPI_I2S_ReceiveData(SPI1);
}

uint16_t mySPI1_ReadData(void){
	/*Tested, hangs with chip select low, but SCK never moves*/
	/*Tested a 2nd time after SPI2 worked and now spi1 works but chip select is always low..***/
	/* It is this version of read that most use to get data from an ADC */
	/* Requires outside Chip Select managment*/
//	SPI1->DR = 0xAA; //Dummy value to start the clock in SCK spi1
//	if(SPI1->SR & SPI_I2S_FLAG_TXE)
	SPI_I2S_SendData(SPI1,0x0000);//this sends the data pattern argument
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE)); //This condition is never met
	while(SPI1->SR & SPI_I2S_FLAG_BSY);
//	return SPI1->DR;
	return SPI_I2S_ReceiveData(SPI1);
}

uint16_t mySPI2_ReadData(void){
/*********TESTED IT WORKS with setting from spi1cfg changed to GPIOB and CS=12****************/

	/* It is this version of read that most use to get data from an ADC */
//	SPI1->DR = 0xAAAA; //Dummy value to start the clock in SCK spi1
	
	SPI_I2S_SendData(SPI2,0xAAAA);
	while(!(SPI2->SR & SPI_I2S_FLAG_TXE));
	while(!(SPI2->SR & SPI_I2S_FLAG_RXNE));
	while(SPI2->SR & SPI_I2S_FLAG_BSY);
//	return SPI1->DR;
	return SPI_I2S_ReceiveData(SPI2);
}

void myDUT_poll(void){
	
/* Read the Dut pins and push their values into the flag holder variable DUTstatus */	
	DUTstatus = (palReadPad(GPIOA, 15) << 7) | (palReadPad(GPIOC, 10) << 6) | (palReadPad(GPIOC, 11) << 5) | (palReadPad(GPIOC, 12) << 4) | (palReadPad(GPIOD, 2) << 3) | (palReadPad(GPIOD, 1) << 2) | (palReadPad(GPIOD, 3) << 1) | (palReadPad(GPIOD, 6) << 0);
/* a second way to do this may be to use ((uint8_t)0x01) = 00000001
 *                                       ((uint8_t)0x02) = 00000010
 *                                       ((uint8_t)0x04) = 00000100
 *                                       ((uint8_t)0x08) = 00001000 ext for bit setting in the data field.
 */
/*

		palClearPad(GPIOD, 6); //DUT8 LSB
		palClearPad(GPIOD, 3);
	  	palClearPad(GPIOD, 1);
		palClearPad(GPIOD, 2);
		palClearPad(GPIOC, 12);
		palClearPad(GPIOC, 11);
		palClearPad(GPIOC, 10);
		palClearPad(GPIOA, 15); //DUT1 MSB
*/
}

/* Cant call the EXTI this way, throws a multiple definition of `VectorE0' error
void EXTI15_10_IRQHandler(void)
{
 CH_IRQ_HANDLER(EXTI15_10_IRQHandler) {} is also defined in ext_lld_isr.c
 EXTD1.config->channels[15].cb(&EXTD1, 15); seems to be where it defines the callback we can put code in


 hal_lld defines EXTI15_10_IRQHandler    VectorE0  < EXTI Line 15..10. >
 it is part of EXTD1

 STD_Perphdriver terms:
 syscfg: SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
 Oursetup for PE15: EXTI_PortSourceGPIOE	EXTI_PinSource15
 Setup in CHibios terms:
 SYSCFG config bits:
 Bit definition for SYSCFG_EXTICR4 register -> SYSCFG_EXTICR4_EXTI15 EXTI 15 configuration
 EXTI15 configuration -> SYSCFG_EXTICR4_EXTI15_PE !<PE[15] pin 
 Exti config bits:
 Bit definition for EXTI_IMR register  -> EXTI_IMR_MR15 Interrupt Mask on line 15
 Bit definition for EXTI_FTSR register -> EXTI_FTSR_TR15 Falling trigger event configuration bit of line 15
 Bit definition for EXTI_PR register   -> EXTI_PR_PR15 Pending bit for line 15
	
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		//interupt detected on line 15 Busy is done, DO SOMETHING like start SPI read
		//Two diff ways to read SPI1. both are here till one is chosen as the winner
	data = SPI1_recv();
	data = mySPI_GetData(0x20); //
	//Reset the bit so another interupt can happen
	EXTI_ClearITPendingBit(EXTI_Line15);
	}
}
*/

//static void extcb15(EXTDriver *extp, expchannel_t channel){
static void extcb15(void){
/********* EXTI 15 CallBack: Watches Busy from ADC Falling edge********/
#if 0
	spiSelect(&SPID1);	
	rxbuf[0]=mySPI1_ReadData();
	rxbuf[1]=mySPI1_ReadData();
	spiUnselect(&SPID1);
#endif
#if 0 //This is the one we use
	spiSelect(&SPID1);	
	cur_reading.current=mySPI1_ReadData();
	cur_reading.voltage=mySPI1_ReadData();
	cur_reading.DUTstate=DUTstatus;
	spiUnselect(&SPID1);
	writebuffer(&cake,cur_reading);
#endif
#if 0
	spiSelect(&SPID1);	
	rxbuf[0]=SPI1_recv();
	rxbuf[1]=SPI1_recv();
	spiUnselect(&SPID1);
#endif
#if 0
	
	mychSPI_recieve();
	
#endif
#if 0	
    palSetPad(GPIOD, GPIOD_LED5);       /* LED ON.                          */
    spiStart(&SPID1, &spi1cfg);         /* Setup transfer parameters.       */
    spiSelect(&SPID1);                  /* Slave Select assertion.          */
    spiStartExchangeI(&SPID1, 2, txbuf, rxbuf);/* Atomic transfer operations.The #is number of transactions total*/
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    palClearPad(GPIOD, GPIOD_LED5);
#endif
#if 0
	if (SDU1.config->usbp->state == USB_ACTIVE) {
		chprintf(&SDU1, "\rCurrent:%u Voltage:%u Dut:%u\n",rxbuf[0], rxbuf[1],DUTstatus);
	}
	palTogglePad(GPIOD, 15);
#endif
	palTogglePad(GPIOD, 15);
#if ADCv1
/*************************EXTI clear for ADCv1*****************************************/
	EXTI_ClearITPendingBit(EXTI_Line15);
#endif
#if ADCv2
/*************************EXTI clear for ADCv1*****************************************/
	EXTI_ClearITPendingBit(EXTI_Line12);
#endif
}
//static void extcb2(EXTDriver *extp, expchannel_t channel){
static void extcb2(void){
palTogglePad(GPIOD, 12);
//EXTI_GenerateSWInterrupt(EXTI_Line12);
EXTI_ClearITPendingBit(EXTI_Line2);
}
/***********************************************************
__attribute__((noreturn)) msg_t EXTIpollthread(void *arg) {
	(void)arg;
	chRegSetThreadName("DUTpoll");
	
	while (TRUE) {
		if(EXTI_GetITStatus(EXTI_Line15) != RESET){
			palSetPad(GPIOD, 13);
			EXTI_ClearITPendingBit(EXTI_Line15);
		}	
		chThdSleepMilliseconds(100);
	}
	return 0;
}
************************************************************/
__attribute__((noreturn)) msg_t DUTpollthread(void *arg) {
//static msg_t DUTpollthread(void *arg) {
	(void)arg;
	chRegSetThreadName("DUTpoll");
#if DUT
RED_OFF;
BLUE_OFF;
GREEN_OFF;
ORANGE_OFF;
	while (TRUE){
	  	palSetPad(GPIOD, 6); //MSB
palTogglePad(GPIOD, 12);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOD, 3);
palTogglePad(GPIOD, 13);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOD, 1);
palTogglePad(GPIOD, 14);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOD, 2);
palTogglePad(GPIOD, 15);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOC, 12);
palTogglePad(GPIOD, 12);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOC, 11);
palTogglePad(GPIOD, 13);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOC, 10);
palTogglePad(GPIOD, 14);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOA, 15); //LSB
palTogglePad(GPIOD, 15);
		chThdSleepMilliseconds(500);
		palClearPad(GPIOD, 6); //DUT8 LSB
		palClearPad(GPIOD, 3);
	  	palClearPad(GPIOD, 1);
		palClearPad(GPIOD, 2);
		palClearPad(GPIOC, 12);
		palClearPad(GPIOC, 11);
		palClearPad(GPIOC, 10);
		palClearPad(GPIOA, 15); //DUT1 MSB
		chThdSleepMilliseconds(500);

	}
#endif	

#if HOST
	while (TRUE) {
	myDUT_poll();
	chThdSleepMilliseconds(100);
	}
#endif
}
/* Might need to create a thread to read SPI as per a post of it having a bug while running with Serial USB */
#if 0
__attribute__((noreturn)) static msg_t SPI1thread(void *p) {

  (void)p;
  chRegSetThreadName("SPI1 thread");
  while (TRUE) {
    spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
    palSetPad(GPIOD, GPIOD_LED5);       /* LED ON.                          */
    spiStart(&SPID1, &spi1cfg);         /* Setup transfer parameters.       */
    spiSelect(&SPID1);                  /* Slave Select assertion.          */
    spiExchange(&SPID1, 1,
                txbuf, rxbuf);          /* Atomic transfer operations.      */
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
  }
  return 0;
}
#endif
void dummyfill(buffer* buff){ //Funtion to load a buffer with 25 values for testing purposes
	int i;
	readings temp;
	for(i=0;i<=25;i++){
		temp.voltage=i+i;
		temp.current=i+2*i;
		temp.DUTstate=i;
		writebuffer(buff,temp);
	}
}
/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

static WORKING_AREA(waInstrumentThread, 4096);
static WORKING_AREA(waDUTpollthread, 128);
//static WORKING_AREA(waEXTIpollthread, 256);
//static WORKING_AREA(waSPI1thread, 256);

/*
 * Application entry point.
 */
int main(void) {
  Thread *shelltp = NULL;

//uint16_t recieved_val = 0; //inital SPI idea
//volatile uint16_t first_adc_val = 0;
//volatile uint16_t second_adc_val = 0;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
//  mySTSPI_Init(); //SPI configuration and pin setup in STMstdLib commands
  myCHSPI_Init(); //SPI configuration and pin setup in ChibiOS commands
  myCHEXTI_Init();
//  mySTEXTI_Init(); //Exti pin and interrupt enable
  myGPIO_Init();
  initBuffer(&cake);
  /* LED GPIO init
   * PD12 - Green, PD13 - Orange, PD14 - Red, PD15 - Blue
   */
  palSetPadMode(GPIOD, 12, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOD, 13, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOD, 14, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOD, 15, PAL_MODE_OUTPUT_PUSHPULL);

  GREEN_OFF;
  ORANGE_OFF;
  RED_ON;
  BLUE_OFF;
  
//Ryan Commented the below out
  /* DEBUG pins used for timing, debug: PB4 == "debug 1", PB5 = "debug 2"
   *   These macros are defined in instr_debug.h
   *       DB1_HI, DB1_LO
   *       DB2_HI, DB2_LO
   */ 
  // DEBUG 1-5 are all on outside consecutive pins
/*palSetPadMode(GPIOE, 7, PAL_MODE_OUTPUT_PUSHPULL);  // DEBUG1
  palSetPadMode(GPIOE, 9, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE,11, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE,13, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE,15, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE, 8, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE,10, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE,12, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE,14, PAL_MODE_OUTPUT_PUSHPULL);  // DEBUG9

  DB1_LO;DB2_LO;DB3_LO;DB4_LO;DB5_LO;DB6_LO;DB7_LO;DB8_LO;DB9_LO; */
  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Initializes a serial-over-USB CDC driver.
   */

  // Bulk usb driver setup
  bduObjectInit(&BDU1);
  bduStart(&BDU1, &blkusbcfg);


  //This one needs to be last so the usb device's upstream pointer is this driver
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  
  
  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /* Ryan edited, not sure what uart is used for but we selected PA2 for connecting to ADC CNVST pin with pwm.
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.  
   * Alternate pins = TX->PD5 RX->PD6
   */
//  sdStart(&SD2, NULL);
//  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
//  palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
//  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
//  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

  /*
   * Initialize I2C #1 Driver. Setup SDA=PB7, SCL=PB8
   */
#ifdef _BBI2C_INCLUDED
  init_bbI2C();

  //Clear and setup the equalizer chip
  hmc6545setup(&equalizer, NULL, 0x1c, "equalizer");
  hmc6545softRst(&equalizer);
  hmc6545clearChip(&equalizer);
  BLUE_ON;0
#endif


  /*
   * Initializes the SPI driver 2. The SPI2 signals are routed as follow:
   * PB12 - NSS.
   * PB13 - SCK.
   * PB14 - MISO.
   * PB15 - MOSI.
   */
  spiStart(&SPID2, &spi2cfg);
  palSetPad(GPIOB, 12);
  palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL |
                PAL_STM32_OSPEED_HIGHEST|PAL_MODE_ALTERNATE(5) );	/* NSS.     */
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5) |
                PAL_STM32_OSPEED_HIGHEST);				/* SCK.     */
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5));			/* MISO.    */
  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5) |
                PAL_STM32_OSPEED_HIGHEST);				/* MOSI.    */


  /*Ryan Added
   *Enables the PWM output (of TIM2, Channel 3) on PA2 connected to ADC CNVST*/
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(1));
  pwmStart(&PWMD2, &pwm2cfg);
  /* function options:what pwm to turn on, which channel to use 0-3, the ON time of the PWM pulse in PWM ticks*/
  pwmEnableChannel(&PWMD2, 2, 1); //was set to 500
  

  /*
   * Creates the Instrument thread to get responces from USB
   */
  chThdCreateStatic(waInstrumentThread, sizeof(waInstrumentThread),
                    NORMALPRIO + 10, InstrumentThread, NULL);

  /* Creates the Thread that checks the status of the DUT GPIO pins*/
  chThdCreateStatic(waDUTpollthread, sizeof(waDUTpollthread),NORMALPRIO + 10, DUTpollthread, NULL);
  /* Creates the Thread that runs the SPI communication */
//USB connect  chThdCreateStatic(waSPI1thread, sizeof(waSPI1thread),NORMALPRIO + 10, SPI1thread, NULL);
  /* Creates the Thread that is a workaround for the EXTI line 15 callback  */
  /* Test with: EXTI_GenerateSWInterrupt(EXTI_Line15);*/
//  chThdCreateStatic(waEXTIpollthread, sizeof(waEXTIpollthread),NORMALPRIO + 15 , EXTIpollthread, NULL);
//test setup for rxbuf[] to debug over SPI1 & load txbuf
rxbuf[5] = 0x0001;
txbuf[0] = 0x0004;
txbuf[1] = 0x6666;
txbuf[2] = 0x8888;
//for (i = 0; i < 512; i++){
//    txbuf[i] = i;}

  /*
   * Normal main() thread activity, in this demo it just performs
   * a shell respawn upon its termination.
   */
  while (TRUE) {
	
/*******************TEST METHOD3************************
    if (palReadPad(GPIOA, GPIOA_BUTTON)){
	chprintf(&SDU1, "USB Data Read\n");
	palTogglePad(GPIOD, 12);
	}
/********************TEST METHOD2*************************
    if (palReadPad(GPIOA, GPIOA_BUTTON)){
	spiSelect(&SPID1);  //chip select low
	SPI1->DR = 0x00;
//	SPI_SR_TXE or SPI_I2S_FLAG_TXE
	if(SPI1->SR & SPI_I2S_FLAG_TXE){
	ORANGE_ON;
	}
	SPI1->DR = 0xAA;
	if(SPI1->SR & SPI_I2S_FLAG_TXE){
	GREEN_ON;
	}
	BLUE_ON;
	chThdSleepMilliseconds(50);
	spiUnselect(&SPID1);
	BLUE_OFF;
	chThdSleepMilliseconds(50);
	spiSelect(&SPID1);  //chip select low
	BLUE_ON;
	chThdSleepMilliseconds(50);
	spiUnselect(&SPID1);
	BLUE_OFF;
    }
*/
/********************TEST METHOD1*************************/  

    if (palReadPad(GPIOA, GPIOA_BUTTON)){
	for(i=0;i<5;i++){
//	cur_reading.current=258;
//	cur_reading.voltage=772;
//	cur_reading.DUTstate=5;
	cur_reading.current=513;
	cur_reading.voltage=1027;
	cur_reading.DUTstate=5;
	writebuffer(&cake,cur_reading);
	}
	palTogglePad(GPIOD, 13);
/*************** SPI x1 read on Button press Test Case********/
#if 0
		ORANGE_OFF;
		palClearPad(GPIOA, 4);  //chip select low
		//ADC sends out ReadingA first then ReadingB.
		// A = Current B = Voltage
		rxbuf[0]=mySPI1_ReadData();
		rxbuf[1]=mySPI1_ReadData();
//		rxbuf[0]=mySPI_GetData(0xBBBB);
//		rxbuf[0]=SPI1_recv();
//		rxbuf[1]=SPI1_recv();
//		mychSPI_recieve();
//		spiStartReceive(&SPID1, 2, rxbuf);
		ORANGE_ON;
		palSetPad(GPIOA, 4);
#endif
//		chprintf(&SDU1, "\r%u\n",DUTstatus);
/******************* Read all circle buffer on button press and send out Serial USB Test Case********/
#if 0
		while(!cake.empty){
			send_reading = readbuffer(&cake);
			chprintf(&SDU1, "\rreading->Current:%u Voltage:%u Dut:%u\n",send_reading.current,send_reading.voltage,send_reading.DUTstate);
		}
#endif
/******************* Read all circle buffer on button press and send out BULK USB Test Case********/
#if 0
		while(!cake.empty){
			send_reading = readbuffer(&cake);
			chprintf(&BDU1, "\rreading->Current:%u Voltage:%u Dut:%u\n",send_reading.current,send_reading.voltage,send_reading.DUTstate);
		}
#endif
/******************* Load Circle buffer with 25 values every button press********/
#if 0
	dummyfill(&cake);
#endif
	}

    if (!shelltp) {
      if (SDU1.config->usbp->state == USB_ACTIVE) {
        /* Spawns a new shell.*/
        shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
	/* Ryan Test to see if i can print to Serial USB from here */
	chprintf(&SDU1, "\r!!All Your Base Are Belong To US!!\n");
//	chprintf(&SDU1, "\r%u\n",DUTstatus);
	chprintf(&SDU1, "\rrxbuf->Current:%u Voltage:%u Dut:%u\n",rxbuf[0], rxbuf[1],DUTstatus);
//	chprintf(&SDU1, "\rreading->Current:%u Voltage:%u Dut:%u\n",cur_reading.current,cur_reading.voltage,cur_reading.DUTstate);
/****************Serial EP1 Data Test case******************/
#if 0
	while (TRUE){
		chprintf(&SDU1, "\rreading->Current:%u Voltage:%u Dut:%u\n",cur_reading.current,cur_reading.voltage,cur_reading.DUTstate);
	chThdSleepMilliseconds(1000);
	}
#endif 
      }
    }
    else {
      /* If the previous shell exited.*/
      if (chThdTerminated(shelltp)) {
        /* Recovers memory of the previous shell.*/
        chThdRelease(shelltp);
        shelltp = NULL;
      }
    }
    chThdSleepMilliseconds(500);
  }
}
#if 0
From GPIOv?
 Pal Modes for GIPO Pins
#define PAL_STM32_MODE_INPUT            (0 << 0)
#define PAL_STM32_MODE_OUTPUT           (1 << 0)
#define PAL_STM32_MODE_ALTERNATE        (2 << 0)
#define PAL_STM32_MODE_ANALOG           (3 << 0)

PAL_MODE_INPUT
PAL_MODE_OUTPUT_PUSHPULL


#define PAL_STM32_OTYPE_MASK            (1 << 2)
#define PAL_STM32_OTYPE_PUSHPULL        (0 << 2)
#define PAL_STM32_OTYPE_OPENDRAIN       (1 << 2)

#define PAL_STM32_OSPEED_MASK           (3 << 3)
#define PAL_STM32_OSPEED_LOWEST         (0 << 3)
#define PAL_STM32_OSPEED_HIGHEST        (3 << 3)

#define PAL_STM32_PUDR_MASK             (3 << 5)
#define PAL_STM32_PUDR_FLOATING         (0 << 5)
#define PAL_STM32_PUDR_PULLUP           (1 << 5)
#define PAL_STM32_PUDR_PULLDOWN         (2 << 5)

#define PAL_STM32_ALTERNATE_MASK        (15 << 7)
#define PAL_STM32_ALTERNATE(n)          ((n) << 7)
#endif
