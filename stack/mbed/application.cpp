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


#ifdef USE_EEPROM
    CO_EE_t                     CO_EEO;         /* Eeprom object */
#endif

#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */
volatile uint16_t   CO_timer1ms = 0U;       /* variable increments each millisecond */
Thread processThread, tmrThread;
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

  tmrThread.start(tmrTask_thread);
  processThread.start(processTask_thread);
}


/*******************************************************************************/
void communicationReset(mbed::CAN *can){
  CO_ReturnError_t err;
  if (can == NULL)
    err = CO_init((int32_t)new mbed::CAN(D15,D14)/* CAN module address */, 1/* NodeID */, 125 /* bit rate */);
  else
    err = CO_init((int32_t)can/* CAN module address */, 1/* NodeID */, 125 /* bit rate */);
  if(err != CO_ERROR_NO) {
      while(1);
      CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
  }
  /* start CAN */
  CO_CANsetNormalMode(CO->CANmodule[0]);
}


/*******************************************************************************/
void programEnd(void){

}

/*******************************************************************************/
void processTask_thread(void) {
  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  uint16_t timer1msPrevious = CO_timer1ms, timerNext_ms = 50;
  Timer t;
    while(1) {
      t.start();
      uint16_t timer1msCopy, timer1msDiff;

      timer1msCopy = CO_timer1ms;
      timer1msDiff = timer1msCopy - timer1msPrevious;
      timer1msPrevious = timer1msCopy;
      reset = CO_process(CO, timer1msDiff, &timerNext_ms);
      //printf("timerNext_ms %d\n", timerNext_ms);
      #ifdef USE_EEPROM
        CO_EE_process(&CO_EEO);
      #endif
      wait_ms(timerNext_ms);
    }
}


/*******************************************************************************/
/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void) {
  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  Timer t;
    while(1) {
      t.start();
      wait_ms(1);
      INCREMENT_1MS(CO_timer1ms);

      if(CO->CANmodule[0]->CANnormal) {
          bool_t syncWas;

          /* Process Sync and read inputs */
          syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);

          /* Further I/O or nonblocking application code may go here. */

          /* Write outputs */
          CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);

          /* verify timer overflow */
          if(0) {
              CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
          }
        }
        t.stop();
        //printf("The time taken was %d seconds\n", t.read_ms());
        t.reset();
    }
}
