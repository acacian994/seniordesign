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

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "cmd_shell.h"
#include "chprintf.h"
#include "instr_cmds.h"
#include "instr_task.h"
#include "usbcmdio.h"



SerialUSBDriver SDU1;
const ShellCommand commands[];
extern usb_packet_t dbgPktBuf;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
             (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
             (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
             states[tp->p_state], (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}


void cmd_id(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  (void)argv;
  MD5_TEK myHash;
  if (argc > 0) {
    chprintf(chp, "Usage: id\r\n");
    return;
  }
  get_instrument_ID (&dbgPktBuf);
  print_ID      (chp,&dbgPktBuf);
  get_instrument_SSN(&dbgPktBuf);
  print_SSN     (chp,&dbgPktBuf);
  myHash = ssn_to_MD5();
  print_UID48   (chp,&myHash);
}

void cmd_konami(BaseSequentialStream *chp, int argc, char *argv[]) 
{/*Ryan added, serial USB command to Flash LEDs */
  (void)argv;
  MD5_TEK myHash;
  int i = 0;
  if (argc > 0) {
    chprintf(chp, "Usage: Konami\r\n");
    return;
  }

palClearPad(GPIOD, 12);
palClearPad(GPIOD, 13);
palClearPad(GPIOD, 14);
palClearPad(GPIOD, 15);

  palSetPad(GPIOD, 13);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 13);
  chThdSleepMilliseconds(75);
  palSetPad(GPIOD, 13);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 13);
  chThdSleepMilliseconds(75);
  palSetPad(GPIOD, 15);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 15);
  chThdSleepMilliseconds(75);
  palSetPad(GPIOD, 15);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 15);
  chThdSleepMilliseconds(75);
  palSetPad(GPIOD, 12);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 12);
  chThdSleepMilliseconds(75);
  palSetPad(GPIOD, 14);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 14);
  chThdSleepMilliseconds(75);
  palSetPad(GPIOD, 12);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 12);
  chThdSleepMilliseconds(75);
  palSetPad(GPIOD, 14);
  chThdSleepMilliseconds(75);
  palClearPad(GPIOD, 14);
  chThdSleepMilliseconds(75);

  for(i;i<=2;i++){
  palSetPad(GPIOD, 12);
  chThdSleepMilliseconds(50);
  palSetPad(GPIOD, 13);
  chThdSleepMilliseconds(50);
  palSetPad(GPIOD, 14);
  chThdSleepMilliseconds(50);
  palSetPad(GPIOD, 15);
  chThdSleepMilliseconds(50);
  palClearPad(GPIOD, 12);
  chThdSleepMilliseconds(50);
  palClearPad(GPIOD, 13);
  chThdSleepMilliseconds(50);
  palClearPad(GPIOD, 14);
  chThdSleepMilliseconds(50);
  palClearPad(GPIOD, 15);
  chThdSleepMilliseconds(50);
  }


}
void cmd_adc(BaseSequentialStream *chp, int argc, char *argv[]) 
{/*Ryan added, serial USB command to get oldest reading from circle buffer and display */
  (void)argv;
  MD5_TEK myHash;
  if (argc > 0) {
    chprintf(chp, "Usage: adc\r\n");
    return;
  }
  get_instrument_ADC(&dbgPktBuf);
  print_ADC(chp,&dbgPktBuf);
  
}


/*
static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(),
                           TestThread, chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }
  chThdWait(tp);
}*/

const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"id", cmd_id},
  {"Konami",cmd_konami},/*Ryan added*/
  {"adc",cmd_adc},/*Ryan added*/
  {NULL, NULL}
};

const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};




