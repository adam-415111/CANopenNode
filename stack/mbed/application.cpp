/*
 * Application interface for CANopenNode stack.
 *
 * @file        application.c
 * @ingroup     application
 * @author      Janez Paternoster
 * @copyright   2012 - 2013 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Following clarification and special exception to the GNU General Public
 * License is included to the distribution terms of CANopenNode:
 *
 * Linking this library statically or dynamically with other modules is
 * making a combined work based on this library. Thus, the terms and
 * conditions of the GNU General Public License cover the whole combination.
 *
 * As a special exception, the copyright holders of this library give
 * you permission to link this library with independent modules to
 * produce an executable, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting
 * executable under terms of your choice, provided that you also meet,
 * for each linked independent module, the terms and conditions of the
 * license of that module. An independent module is a module which is
 * not derived from or based on this library. If you modify this
 * library, you may extend this exception to your version of the
 * library, but you are not obliged to do so. If you do not wish
 * to do so, delete this exception statement from your version.
 */

#include "application.h"

#if defined(TARGET_STM32F303xB)
  #define CANRD CAN_RD
  #define CANRT CAN_TD
#else
  #define CANRD D15
  #define CANRT D14
#endif
#if defined(TARGET_STM32F334R8) || defined(TARGET_STM32F303K8)
  #undef CANRD
  #undef CANRT
  #define CANRD PA_11
  #define CANRT PA_12
#endif

CO_NMT_reset_cmd_t reset = CO_RESET_NOT;

#ifdef USE_EEPROM
    CO_EE_t                     CO_EEO;         /* Eeprom object */
#endif
//#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */
//volatile uint16_t   CO_timer1ms = 0U;       /* variable increments each millisecond */
//Thread processThread, tmrThread;
/*******************************************************************************/
void programStart(mbed::CAN *can){
  /* initialize EEPROM - part 1 */
#ifdef USE_EEPROM
  CO_ReturnError_t eeStatus = CO_EE_init_1(&CO_EEO, (uint8_t*) &CO_OD_EEPROM,
        sizeof(CO_OD_EEPROM)/*, (uint8_t*) &CO_OD_ROM, sizeof(CO_OD_ROM)*/);
#endif

  communicationReset(can);

  /* initialize eeprom - part 2 */
#ifdef USE_EEPROM
   CO_EE_init_2(&CO_EEO, eeStatus, CO->SDO[0], CO->em);
#endif

  //queue.call_every(50, tmrTask_thread);
  //queue.call_every(50, processTask_thread);

}


/*******************************************************************************/
void communicationReset(mbed::CAN *can){
  CO_ReturnError_t err;

  if (can == NULL)
    can = new mbed::CAN(CANRD,CANRT);
    //printf("OD_CANBitRate: %d\r\n", OD_CANBitRate);
  can->frequency(OD_CANBitRate*1000);
  can->filter(0x000, 0x700, CANStandard);
  //printf("handle: %d\r\n",handle);
  can->filter(0x100, 0x780, CANStandard, 1);
  //printf("handle: %d\r\n",handle);
  can->filter(0x580, 0x780, CANStandard, 2);
  //printf("handle: %d\r\n",handle);
  can->filter(0x600, 0x700, CANStandard, 3);
  //printf("handle: %d\r\n",handle);
  can->filter(0x700, 0x700, CANStandard, 4);
  //printf("handle: %d\r\n",handle);
  /*handle = can->filter(0x180, 0x780, CANStandard, handle);
  handle = can->filter(0x200, 0x7FF, CANStandard, handle);
  handle = can->filter(0x300, 0x7FF, CANStandard, handle);
  handle = can->filter(0x400, 0x7FF, CANStandard, handle);
  handle = can->filter(0x570, 0x780, CANStandard, handle);*/
  err = CO_init((int32_t)can, OD_CANNodeID, OD_CANBitRate);

  if(err != CO_ERROR_NO) {
      CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
  }
  /* start CAN */
  CO_CANsetNormalMode(CO->CANmodule[0]);
}


/*******************************************************************************/
void programEnd(void){

}

Timer t;
uint16_t timerNext_ms = 1;
#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
/*******************************************************************************/
void processTask_thread(void) {
  //CO_NMT_reset_cmd_t reset = CO_RESET_NOT;

  //Timer t;
  //t.start();
  //while(1) {
    //printf("timerNext_ms: %d\n", timerNext_ms);
    //printf("CO->NMT->operatingState: %d\n", CO->NMT->operatingState);
    reset = CO_process(CO, t.read_ms(), NULL);
    //t.reset();
    #ifdef USE_EEPROM
      CO_EE_process(&CO_EEO);
    #endif

  // Check for the different commands
  if (reset == CO_RESET_COMM) {
    //Communication reset should reset the can bus and clean the error but can.reset does not work.
    //Device software reset also blocks for devices f344
    CO_errorReset(CO->em, CO_EM_CAN_TX_BUS_OFF, 0);
    CO_errorReset(CO->em, CO_EM_CAN_TX_BUS_PASSIVE, 0);
    CO_errorReset(CO->em, CO_EM_CAN_TX_OVERFLOW, 0);
    CO_errorReset(CO->em, CO_EM_CAN_BUS_WARNING, 0);
    CO_errorReset(CO->em, CO_EM_CAN_RX_BUS_PASSIVE, 0);
    CO_errorReset(CO->em, CO_EM_CAN_TX_OVERFLOW, 0);
    for (unsigned char i = 0; i < CO->em->errorStatusBitsSize; i++)
      CO->em->errorStatusBits[i] = 0;
    (*CO->NMT->emPr->errorRegister) = 0U;

    reset == CO_RESET_NOT;
    CO->NMT->resetCommand = CO_NMT_PRE_OPERATIONAL;

  } else if (reset == CO_RESET_APP) {
      NVIC_SystemReset();
  } else if (reset == CO_RESET_QUIT) {
      NVIC_SystemReset();
  }
  //printf("timerNext_ms after: %d\n", timerNext_ms);
  //printf("processTask_thread end\n");
  /*t.stop();
  printf("processTask_thread: %d milliseconds\n", t.read_ms());*/
  t.reset();
  t.start();
  return;
}

/*******************************************************************************/
/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void) {
  //Timer t;
  //t.start();
  //CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  //uint16_t timerNext_ms = 50;
    //while(1) {
      //INCREMENT_1MS(CO_timer1ms);
      if(CO->CANmodule[0]->CANnormal) {
          bool_t syncWas;
          /* Process Sync and read inputs */
          syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);
          /* Further I/O or nonblocking application code may go here. */
          /* Write outputs */
          CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);
          /* verify timer overflow */
          //if(0) {
          //    CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
          //}
        }
        //wait_ms(1);
        //t.stop();
        //printf("tmrTask_thread: %d milliseconds\n", t.read_ms());
        //t.reset();
    //}
    //t.stop();
    //printf("tmrTask_thread: %d milliseconds\n", t.read_ms());
    //t.reset();
}
