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

#include <application.h>
#include <stdint.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>


#ifdef USE_STORAGE
    static CO_OD_storage_t      odStor; /* Object Dictionary storage object for CO_OD_ROM */
    static CO_OD_storage_t      odStorAuto;         /* Object Dictionary storage object for CO_OD_EEPROM */
    static char                *odStorFile_rom    = "od_storage";       /* Name of the file */
    static char                *odStorFile_eeprom = "od_storage_auto";  /* Name of the file */
    CO_ReturnError_t odStorStatus_rom, odStorStatus_eeprom;
    //CO_EE_t                     CO_EEO;         /* Eeprom object */
#endif

#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */
volatile uint16_t   CO_timer1ms = 0U;       /* variable increments each millisecond */
boost::thread *processThread, *tmrThread;

extern SerialPort *_port = NULL;

/* Signal handler */
volatile sig_atomic_t CO_endProgram = 0;
/*static void sigHandler(int sig) {
    CO_endProgram = 1;
}*/

/* Helper functions ***********************************************************/
/*void CO_errExit(char* msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}*/

/* send CANopen generic emergency message */
/*void CO_error(const uint32_t info) {
    CO_errorReport(CO->em, CO_EM_GENERIC_SOFTWARE_ERROR, CO_EMC_SOFTWARE_INTERNAL, info);
    fprintf(stderr, "canopend generic error: 0x%X\n", info);
}*/


/*******************************************************************************/
void programStart(SerialPort *serial) {
    //bool firstRun = true;

#ifdef USE_STORAGE
    odStorFile_rom = "rom.txt";
    odStorFile_eeprom = "eeprom.txt";

    /* Verify, if OD structures have proper alignment of initial values */
    if(CO_OD_RAM.FirstWord != CO_OD_RAM.LastWord) {
	fprintf(stderr, "Program init - %s - Error in CO_OD_RAM.\n", odStorFile_rom);
	exit(EXIT_FAILURE);
    }
    if(CO_OD_EEPROM.FirstWord != CO_OD_EEPROM.LastWord) {
	fprintf(stderr, "Program init - %s - Error in CO_OD_EEPROM.\n", odStorFile_eeprom);
	exit(EXIT_FAILURE);
    }
    if(CO_OD_ROM.FirstWord != CO_OD_ROM.LastWord) {
	fprintf(stderr, "Program init - %s - Error in CO_OD_ROM.\n", odStorFile_rom);
	exit(EXIT_FAILURE);
    }

    /* initialize Object Dictionary storage */
    odStorStatus_rom = CO_OD_storage_init(&odStor, (uint8_t*) &CO_OD_ROM, sizeof(CO_OD_ROM), odStorFile_rom);
    odStorStatus_eeprom = CO_OD_storage_init(&odStorAuto, (uint8_t*) &CO_OD_EEPROM, sizeof(CO_OD_EEPROM), odStorFile_eeprom);

#endif
    /* Catch signals SIGINT and SIGTERM */
    /*if(signal(SIGINT, sigHandler) == SIG_ERR)
	CO_errExit("Program init - SIGINIT handler creation failed");
    if(signal(SIGTERM, sigHandler) == SIG_ERR)
	CO_errExit("Program init - SIGTERM handler creation failed");*/

    /* increase variable each startup. Variable is automatically stored in non-volatile memory. */
    printf(", count=%u ...\n", ++OD_powerOnCounter);

  communicationReset(serial);

  /* initialize OD objects 1010 and 1011 and verify errors. */
    CO_OD_configure(CO->SDO[0], OD_H1010_STORE_PARAM_FUNC, CO_ODF_1010, (void*)&odStor, 0, 0U);
    CO_OD_configure(CO->SDO[0], OD_H1011_REST_PARAM_FUNC, CO_ODF_1011, (void*)&odStor, 0, 0U);
    if(odStorStatus_rom != CO_ERROR_NO) {
	CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_rom);
    }
    if(odStorStatus_eeprom != CO_ERROR_NO) {
	CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_eeprom + 1000);
    }


  tmrThread = new boost::thread(tmrTask_thread);
  processThread = new boost::thread(processTask_thread);

    //reset = CO_RESET_NOT;

    printf("%s - running ...\n"/*, argv[0]*/);
}


/*******************************************************************************/
void communicationReset(SerialPort *serial) {
  CO_ReturnError_t err;
  if (serial == NULL)
      serial = new SerialPort();
    err = CO_init( *((int32_t*)serial)/* CAN module address */, CO_OD_ROM.CANNodeID /*1 NodeID */, CO_OD_ROM.CANBitRate /*125 bit rate */);
  if(err != CO_ERROR_NO) {
      while(1);
      CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
  }
    _port = serial;

    if (!_port->start("/dev/ttyACM0", 9600)) {
	printf("BAD SERIAL.\n");
	return;
    }

  _port->async_read_some_(on_receive_can);


  /* start CAN */
  CO_CANsetNormalMode(CO->CANmodule[0]);
}


/*******************************************************************************/
void programEnd(void){

}

/*******************************************************************************/
void processTask_thread(void) {
    std::cout << "processTask_thread" << std::endl;
  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  uint16_t timer1msPrevious = CO_timer1ms, timerNext_ms = 50;
  //Timer t;
    while(1) {
      //t.start();
      uint16_t timer1msCopy, timer1msDiff;

      timer1msCopy = CO_timer1ms;
      timer1msDiff = timer1msCopy - timer1msPrevious;
      timer1msPrevious = timer1msCopy;
      reset = CO_process(CO, timer1msDiff, &timerNext_ms);
      //printf("timerNext_ms %d\n", timerNext_ms);
      /*#ifdef USE_STORAGE
	CO_EE_process(&CO_EEO);
      #endif*/
      boost::this_thread::sleep(boost::posix_time::milliseconds(timerNext_ms));
    }
}


/*******************************************************************************/
/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void) {
    std::cout << "tmrTask_thread" << std::endl;
  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  //Timer t;
    while(1) {
      //t.start();
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      INCREMENT_1MS(CO_timer1ms);

      if(CO->CANmodule[0]->CANnormal) {
          bool_t syncWas;

	  // Process Sync and read inputs
          syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);

	  // Further I/O or nonblocking application code may go here.

	  // Write outputs
          CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);

	  // verify timer overflow
          if(0) {
              CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
          }
        }
	//t.stop();
        //printf("The time taken was %d seconds\n", t.read_ms());
	//t.reset();
    }
}

void on_receive_can(const std::string &data) {
    std::vector<std::string> strs;
    boost::split(strs,data,boost::is_any_of(" "));
    //std::cout << data << std::endl;
    //std::cout << "SIZE split: " << strs.size() << std::endl;

    CO_CANrxMsg_t rcvMsg;
    if (strs.size() == 12) {
	//strs.at<unsigned short>(0); //CANFormat -> CANStandard = 0, CANExtended = 1
	//strs.at<unsigned short>(1); //CANType -> CANData   = 0, CANRemote = 1
	try {
	    rcvMsg.ident = boost::lexical_cast<unsigned int>(strs[2]);
	    rcvMsg.DLC = boost::lexical_cast<unsigned short>(strs[3]);
	    //std::cout << "id: " << rcvMsg.ident << " -- " << boost::lexical_cast<unsigned int>(strs[2]) << std::endl;
	    //std::cout << "length: " << rcvMsg.DLC << " -- " << boost::lexical_cast<unsigned short>(strs[3]) << std::endl;
	    for (unsigned char i=0; i<rcvMsg.DLC; i++) {
		rcvMsg.data[i] = boost::lexical_cast<unsigned short>(strs[i+2]);
		//std::cout << "data[" << i << "]: " << rcvMsg.data[i] << " -- " << boost::lexical_cast<unsigned short>(strs[i+2]) << std::endl;
	    }
	} catch( boost::bad_lexical_cast const& ) {
	    std::cout << "Error: input string was not valid" << std::endl;
	}
	std::cout << "READ:-->" << data << std::endl;
	CO_CANinterrupt_Rx(&rcvMsg);
    } else
	std::cout << "Error: input string was not valid" << std::endl;
}
