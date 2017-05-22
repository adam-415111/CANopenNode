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
#include <boost/thread.hpp>
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
//#include <easylogging++.h>
//#include <boost/log/trivial.hpp>
#include <data/logger.hpp>
#include <net/if.h>
#include <sys/epoll.h>
#include <pthread.h>
#include <csignal>
=======
#include <easylogging++.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <pthread.h>
>>>>>>> Moved to socketCAN
=======
//#include <easylogging++.h>
#include <boost/log/trivial.hpp>
#include <net/if.h>
#include <sys/epoll.h>
#include <pthread.h>
#include <csignal>
>>>>>>> changed log

#define CO_SDO_BUFFER_SIZE    889
CO_NMT_reset_cmd_t reset_NMT = CO_RESET_NOT;

#ifdef USE_STORAGE
static CO_OD_storage_t      odStor; /* Object Dictionary storage object for CO_OD_ROM */
static CO_OD_storage_t      odStorAuto;         /* Object Dictionary storage object for CO_OD_EEPROM */
static char                *odStorFile_rom    = "od_storage";       /* Name of the file */
static char                *odStorFile_eeprom = "od_storage_auto";  /* Name of the file */
CO_ReturnError_t odStorStatus_rom = CO_ERROR_NO, odStorStatus_eeprom = CO_ERROR_NO;
//CO_EE_t                     CO_EEO;         /* Eeprom object */
#endif

#define TMR_TASK_INTERVAL   (50000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */
volatile uint16_t   CO_timer1ms = 0U;       /* variable increments each millisecond */
boost::thread *processThread = NULL, *tmrThread = NULL;

int CANdevice0Index = 0;
//extern boost::shared_ptr<SerialPort> _port = NULL;
//void (*InterEmergSignal)(void) = NULL;

#define TMR_TASK_INTERVAL_NS    (1000000)       /* Interval of taskTmr in nanoseconds */
#define TMR_TASK_OVERFLOW_US    (5000)          /* Overflow detect limit for taskTmr in microseconds */
static int                  rtPriority = -1;    /* Real time priority, configurable by arguments. (-1=RT disabled) */
static int                  mainline_epoll_fd;  /* epoll file descriptor for mainline */
static void*                rt_thread(void* arg);
static pthread_t            rt_thread_id;
static int                  rt_thread_epoll_fd;
pthread_mutex_t             CO_CAN_VALID_mtx = PTHREAD_MUTEX_INITIALIZER;
static CO_time_t            CO_time;            /* Object for current time */


/* Signal handler */
volatile sig_atomic_t CO_endProgram = 0;
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
/*static void sigHandler(int sig) {
    CO_endProgram = 1;
    CO_exit();
    exit(EXIT_SUCCESS);
}*/

/* Helper functions ***********************************************************/
void CO_exit() {
    DEBUG << "CO_Exit Called";

    reset_NMT = CO_RESET_QUIT;
    CO_endProgram = 1;
    if (!CO) {
        BOOST_LOG_TRIVIAL(info) << "Program end - CanOpen not started";
        return;
        //CO_errExit("Program end - CanOpen not started");
    }
    CO->CANmodule[0]->CANnormal = false;

=======
static void sigHandler(int sig) {
    CO_endProgram = 1;
    CO_exit();
    exit(EXIT_SUCCESS);
}

/* Helper functions ***********************************************************/
void CO_exit() {
    BOOST_LOG_TRIVIAL(debug) << "CO_Exit Called";

    reset_NMT = CO_RESET_QUIT;
    CO_endProgram = 1;
    if (!CO) {
        BOOST_LOG_TRIVIAL(info) << "Program end - CanOpen not started";
        return;
        //CO_errExit("Program end - CanOpen not started");
    }
    CO->CANmodule[0]->CANnormal = false;
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
>>>>>>> Moved to socketCAN
=======

>>>>>>> changed log
    if (tmrThread) {
        tmrThread->join();
    }
    delete tmrThread;
    tmrThread = NULL;

<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    if(pthread_join(rt_thread_id, NULL) != 0) {
        CO_errExit("Program end - pthread_join failed");
    }
    DEBUG << "rt_thread_id done!";
=======

    if(pthread_join(rt_thread_id, NULL) != 0) {
        CO_errExit("Program end - pthread_join failed");
    }
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
    LOG(DEBUG) << "rt_thread_id done!";
>>>>>>> Moved to socketCAN
=======
    BOOST_LOG_TRIVIAL(debug) << "rt_thread_id done!";
>>>>>>> changed log

    /* delete objects from memory */
    CANrx_taskTmr_close();
    taskMain_close();

<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7

    //CANrx_taskTmr_close();
    //taskMain_close();

=======
>>>>>>> Moved to socketCAN
    /*if (processThread) {
        processThread->join();
    }
    delete processThread;
    processThread = NULL;*/

    //if (_port != NULL && _port.get() != NULL) {
    //    _port->stop();
    //}
    boost::this_thread::sleep(boost::posix_time::milliseconds(200));

    CO_delete( CANdevice0Index );
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    DEBUG << "CanOpen closed";
}



void CO_errExit(char* msg) {
    ERROR << msg;
    //perror(msg);
    //CO_exit();
    //exit(EXIT_FAILURE);
=======
    LOG(INFO) << "CanOpen and serial closed";
=======
    BOOST_LOG_TRIVIAL(debug) << "CanOpen closed";
>>>>>>> changed log
}

void CO_errExit(char* msg) {
    perror(msg);
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
    CO_exit();
    exit(EXIT_FAILURE);
>>>>>>> Moved to socketCAN
=======
    //CO_exit();
    //exit(EXIT_FAILURE);
>>>>>>> changed log
}

/* send CANopen generic emergency message */
void CO_error(const uint32_t info) {
    CO_errorReport(CO->em, CO_EM_GENERIC_SOFTWARE_ERROR, CO_EMC_SOFTWARE_INTERNAL, info);
    fprintf(stderr, "canopend generic error: 0x%X\n", info);
}

/*******************************************************************************/
int startCO(std::string CANdevice) {

    if (CO != NULL) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
        DEBUG << "Reseting CO...";
        CO_exit();
=======
	LOG(INFO) << "Reseting CO...";
	CO_exit();
>>>>>>> Moved to socketCAN
=======
        BOOST_LOG_TRIVIAL(debug) << "Reseting CO...";
        CO_exit();
>>>>>>> changed log
    }

    CANdevice0Index = if_nametoindex(CANdevice.c_str());
    if(CANdevice0Index == 0) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
        ERROR << "Can't find CAN device " << CANdevice;
=======
        BOOST_LOG_TRIVIAL(error) << "Can't find CAN device " << CANdevice;
>>>>>>> changed log
        return 11;
        /*char s[120];
        snprintf(s, 120, "Can't find CAN device \"%s\"", CANdevice);
        CO_errExit(s);*/
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
=======
	    char s[120];
	    snprintf(s, 120, "Can't find CAN device \"%s\"", CANdevice);
	    CO_errExit(s);
>>>>>>> Moved to socketCAN
=======
>>>>>>> changed log
    }

    reset_NMT = CO_RESET_NOT;
    CO_endProgram = 0;
#ifdef USE_STORAGE
    odStorFile_rom = "rom.bin";
    odStorFile_eeprom = "eeprom.bin";

    LOG(DEBUG) << "Starting CANopen device with Node ID: " << (int)OD_CANNodeID << "(0x" << std::hex << (int)CO_OD_ROM.CANNodeID << ")";
    //el::Loggers::getLogger("default")->debug("Starting CANopen device with Node ID %d(0x%02X)...\n", OD_CANNodeID, CO_OD_ROM.CANNodeID);

    // Verify, if OD structures have proper alignment of initial values
    if(CO_OD_RAM.FirstWord != CO_OD_RAM.LastWord) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
        LOG(ERROR) << "Error in CO_OD_RAM: " << odStorFile_rom;
        //fprintf(stderr, "Program init - %s - Error in CO_OD_RAM.\n", odStorFile_rom);
        exit(EXIT_FAILURE);
    }

    if(CO_OD_EEPROM.FirstWord != CO_OD_EEPROM.LastWord) {
        LOG(ERROR) << "Error in CO_OD_EEPROM: " << odStorFile_eeprom;
        //fprintf(stderr, "Program init - %s - Error in CO_OD_EEPROM.\n", odStorFile_eeprom);
        exit(EXIT_FAILURE);
    }

    if(CO_OD_ROM.FirstWord != CO_OD_ROM.LastWord) {
        LOG(ERROR) << "Error in Error in CO_OD_ROM: " << odStorFile_rom;
        //fprintf(stderr, "Program init - %s - Error in CO_OD_ROM.\n", odStorFile_rom);
        exit(EXIT_FAILURE);
=======
	LOG(ERROR) << "Error in CO_OD_RAM: " << odStorFile_rom;
	//fprintf(stderr, "Program init - %s - Error in CO_OD_RAM.\n", odStorFile_rom);
	exit(EXIT_FAILURE);
=======
        LOG(ERROR) << "Error in CO_OD_RAM: " << odStorFile_rom;
        //fprintf(stderr, "Program init - %s - Error in CO_OD_RAM.\n", odStorFile_rom);
        exit(EXIT_FAILURE);
>>>>>>> changed log
    }

    if(CO_OD_EEPROM.FirstWord != CO_OD_EEPROM.LastWord) {
        LOG(ERROR) << "Error in CO_OD_EEPROM: " << odStorFile_eeprom;
        //fprintf(stderr, "Program init - %s - Error in CO_OD_EEPROM.\n", odStorFile_eeprom);
        exit(EXIT_FAILURE);
    }

    if(CO_OD_ROM.FirstWord != CO_OD_ROM.LastWord) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
	LOG(ERROR) << "Error in Error in CO_OD_ROM: " << odStorFile_rom;
	//fprintf(stderr, "Program init - %s - Error in CO_OD_ROM.\n", odStorFile_rom);
	exit(EXIT_FAILURE);
>>>>>>> Moved to socketCAN
=======
        LOG(ERROR) << "Error in Error in CO_OD_ROM: " << odStorFile_rom;
        //fprintf(stderr, "Program init - %s - Error in CO_OD_ROM.\n", odStorFile_rom);
        exit(EXIT_FAILURE);
>>>>>>> changed log
    }

    /* initialize Object Dictionary storage */
    //std::cout << "odStorStatus_rom: " << odStorStatus_rom << std::endl;
    //std::cout << "odStorStatus_eeprom: " << odStorStatus_eeprom << std::endl;
    odStorStatus_rom = CO_OD_storage_init(&odStor, (uint8_t*) &CO_OD_ROM, sizeof(CO_OD_ROM), odStorFile_rom);
    odStorStatus_eeprom = CO_OD_storage_init(&odStorAuto, (uint8_t*) &CO_OD_EEPROM, sizeof(CO_OD_EEPROM), odStorFile_eeprom);

#endif

    /* Wait other threads (command interface). */
    pthread_mutex_lock(&CO_CAN_VALID_mtx);

    // Catch signals SIGINT and SIGTERM should close connections....
    //if(signal(SIGINT, sigHandler) == SIG_ERR)
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    //CO_errExit("Program init - SIGINIT handler creation failed");

    //if(signal(SIGTERM, sigHandler) == SIG_ERR)
    //CO_errExit("Program init - SIGTERM handler creation failed");
=======
	//CO_errExit("Program init - SIGINIT handler creation failed");

    //if(signal(SIGTERM, sigHandler) == SIG_ERR)
	//CO_errExit("Program init - SIGTERM handler creation failed");
>>>>>>> Moved to socketCAN
=======
    //CO_errExit("Program init - SIGINIT handler creation failed");

    //if(signal(SIGTERM, sigHandler) == SIG_ERR)
    //CO_errExit("Program init - SIGTERM handler creation failed");
>>>>>>> changed log


    /* increase variable each startup. Variable is automatically stored in non-volatile memory. */
    //printf(", count=%u ...\n", ++OD_powerOnCounter);

    if (communicationStart() < 0) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
        ERROR << "Communication failed";
        //CO_exit();
        return 10;
        //CO_errExit("Serial communication failed");
=======
	LOG(ERROR) << "Serial communication failed";
	//CO_exit();
	return 10;
	//CO_errExit("Serial communication failed");
>>>>>>> Moved to socketCAN
=======
        BOOST_LOG_TRIVIAL(error) << "Communication failed";
        //CO_exit();
        return 10;
        //CO_errExit("Serial communication failed");
>>>>>>> changed log
    }

#ifdef USE_STORAGE
    /* initialize OD objects 1010 and 1011 and verify errors. */
    CO_OD_configure(CO->SDO[0], OD_H1010_STORE_PARAM_FUNC, CO_ODF_1010, (void*)&odStor, 0, 0U);
    CO_OD_configure(CO->SDO[0], OD_H1011_REST_PARAM_FUNC, CO_ODF_1011, (void*)&odStor, 0, 0U);

    if(odStorStatus_rom != CO_ERROR_NO) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
        std::cout << "odStorStatus_rom: " << odStorStatus_rom << std::endl;
        CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_rom);
    }

    if(odStorStatus_eeprom != CO_ERROR_NO) {
        std::cout << "odStorStatus_eeprom: " << odStorStatus_eeprom << std::endl;
        CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_eeprom + 1000);
=======
	std::cout << "odStorStatus_rom: " << odStorStatus_rom << std::endl;
	CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_rom);
    }

    if(odStorStatus_eeprom != CO_ERROR_NO) {
	std::cout << "odStorStatus_eeprom: " << odStorStatus_eeprom << std::endl;
	CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_eeprom + 1000);
>>>>>>> Moved to socketCAN
=======
        std::cout << "odStorStatus_rom: " << odStorStatus_rom << std::endl;
        CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_rom);
    }

    if(odStorStatus_eeprom != CO_ERROR_NO) {
        std::cout << "odStorStatus_eeprom: " << odStorStatus_eeprom << std::endl;
        CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)odStorStatus_eeprom + 1000);
>>>>>>> changed log
    }

#endif
    //boost::this_thread::sleep(boost::posix_time::milliseconds(1));

    /* Initialize time */
    CO_time_init(&CO_time, CO->SDO[0], &OD_time.epochTimeBaseMs, &OD_time.epochTimeOffsetMs, 0x2130);

    /* Configure epoll for mainline */
    mainline_epoll_fd = epoll_create(4);
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    if(mainline_epoll_fd == -1) {
        ERROR << "Program init - epoll_create mainline failed";
        return 12;
        //CO_errExit("Program init - epoll_create mainline failed");
    }
=======
    if(mainline_epoll_fd == -1)
	CO_errExit("Program init - epoll_create mainline failed");
>>>>>>> Moved to socketCAN
=======
    if(mainline_epoll_fd == -1) {
        BOOST_LOG_TRIVIAL(error) << "Program init - epoll_create mainline failed";
        return 12;
        //CO_errExit("Program init - epoll_create mainline failed");
    }
>>>>>>> changed log
    /* Init mainline */
    taskMain_init(mainline_epoll_fd, &OD_performance[ODA_performance_mainCycleMaxTime]);

    /* Configure epoll for rt_thread */
    rt_thread_epoll_fd = epoll_create(2);
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    if(rt_thread_epoll_fd == -1) {
        ERROR << "Program init - epoll_create rt_thread failed";
        return 12;
        //CO_errExit("Program init - epoll_create rt_thread failed");
    }
=======
    if(rt_thread_epoll_fd == -1)
	CO_errExit("Program init - epoll_create rt_thread failed");
>>>>>>> Moved to socketCAN
=======
    if(rt_thread_epoll_fd == -1) {
        BOOST_LOG_TRIVIAL(error) << "Program init - epoll_create rt_thread failed";
        return 12;
        //CO_errExit("Program init - epoll_create rt_thread failed");
    }
>>>>>>> changed log

    /* Init taskRT */
    CANrx_taskTmr_init(rt_thread_epoll_fd, TMR_TASK_INTERVAL_NS, &OD_performance[ODA_performance_timerCycleMaxTime]);

    OD_performance[ODA_performance_timerCycleTime] = TMR_TASK_INTERVAL_NS/1000; /* informative */

    /* Create rt_thread */
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    if(pthread_create(&rt_thread_id, NULL, rt_thread, NULL) != 0) {
        ERROR << "Program init - rt_thread creation failed";
        return 12;
        //CO_errExit("Program init - rt_thread creation failed");
    }

    /* Set priority for rt_thread */
    if(rtPriority > 0) {
        struct sched_param param;

        param.sched_priority = rtPriority;
        if(pthread_setschedparam(rt_thread_id, SCHED_FIFO, &param) != 0) {
            ERROR << "Program init - rt_thread set scheduler failed";
            return 12;
            //CO_errExit("Program init - rt_thread set scheduler failed");
        }
=======
    if(pthread_create(&rt_thread_id, NULL, rt_thread, NULL) != 0)
	CO_errExit("Program init - rt_thread creation failed");

    /* Set priority for rt_thread */
    if(rtPriority > 0) {
	struct sched_param param;

	param.sched_priority = rtPriority;
	if(pthread_setschedparam(rt_thread_id, SCHED_FIFO, &param) != 0)
	    CO_errExit("Program init - rt_thread set scheduler failed");
>>>>>>> Moved to socketCAN
=======
    if(pthread_create(&rt_thread_id, NULL, rt_thread, NULL) != 0) {
        BOOST_LOG_TRIVIAL(error) << "Program init - rt_thread creation failed";
        return 12;
        //CO_errExit("Program init - rt_thread creation failed");
    }

    /* Set priority for rt_thread */
    if(rtPriority > 0) {
        struct sched_param param;

        param.sched_priority = rtPriority;
        if(pthread_setschedparam(rt_thread_id, SCHED_FIFO, &param) != 0) {
            BOOST_LOG_TRIVIAL(error) << "Program init - rt_thread set scheduler failed";
            return 12;
            //CO_errExit("Program init - rt_thread set scheduler failed");
        }
>>>>>>> changed log
    }

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule[0]);

    pthread_mutex_unlock(&CO_CAN_VALID_mtx);

    tmrThread = new boost::thread(tmrTask_main);
    //tmrThread = new boost::thread(tmrTask_thread);
    //processThread = new boost::thread(processTask_thread);

#ifdef USE_STORAGE
    //reset = CO_RESET_NOT;
    //CO_OD_storage_autoSave(&odStorAuto, CO_timer1ms, 60000);
#endif

<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    DEBUG << "...done";
=======
    LOG(DEBUG) << "...done";
>>>>>>> Moved to socketCAN
=======
    BOOST_LOG_TRIVIAL(debug) << "...done";
>>>>>>> changed log
    return 0;
}

/*******************************************************************************/
int communicationStart() {
    CO_ReturnError_t err= CO_ERROR_NO;

<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    err = CO_init( CANdevice0Index, OD_CANNodeID, OD_CANBitRate);
    if(err != CO_ERROR_NO) {
        ERROR << "Failed CO_init: " << err;
        //TODO report to whom
        //CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
        return -1;
        //CO_errExit("Failed CO_init");
    }

    // start CAN
    //CO_CANsetNormalMode(CO->CANmodule[0]);
    DEBUG << "...done.";
=======
    //if (CanDevIndex == NULL)
	//CanDevIndex = new SerialPort();

=======
>>>>>>> changed log
    err = CO_init( CANdevice0Index, OD_CANNodeID, OD_CANBitRate);
    if(err != CO_ERROR_NO) {
        BOOST_LOG_TRIVIAL(error) << "Failed CO_init: " << err;
        //TODO report to whom
        //CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
        return -1;
        //CO_errExit("Failed CO_init");
    }

    // start CAN
    //CO_CANsetNormalMode(CO->CANmodule[0]);
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
    LOG(DEBUG) << "...done.";
>>>>>>> Moved to socketCAN
=======
    BOOST_LOG_TRIVIAL(debug) << "...done.";
>>>>>>> changed log
    return 0;
}

/*******************************************************************************/
void communicationReset() {
    /*CO_ReturnError_t err= CO_ERROR_NO;
    if (serial == NULL)
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    serial = new SerialPort();
    err = CO_init( *((int32_t*)serial), OD_CANNodeID , OD_CANBitRate );
    std::cout << "CO_init: " << err << std::endl;
    if(err != CO_ERROR_NO) {
    //while(1);
    CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
    exit(0);
=======
	serial = new SerialPort();
    err = CO_init( *((int32_t*)serial), OD_CANNodeID , OD_CANBitRate );
    std::cout << "CO_init: " << err << std::endl;
    if(err != CO_ERROR_NO) {
	//while(1);
	CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
	exit(0);
>>>>>>> Moved to socketCAN
=======
    serial = new SerialPort();
    err = CO_init( *((int32_t*)serial), OD_CANNodeID , OD_CANBitRate );
    std::cout << "CO_init: " << err << std::endl;
    if(err != CO_ERROR_NO) {
    //while(1);
    CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
    exit(0);
>>>>>>> changed log
    }
    _port = serial;
    _port->end_of_line_char('\n');
    _port->_func = on_receive_can;
    if (!_port->start("/dev/ttyACM0", 115200)) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    printf("BAD SERIAL.\n");
    return;
=======
	printf("BAD SERIAL.\n");
	return;
>>>>>>> Moved to socketCAN
=======
    printf("BAD SERIAL.\n");
    return;
>>>>>>> changed log
    }
    //_port->async_read_some_(on_receive_can);


    // start CAN
    CO_CANsetNormalMode(CO->CANmodule[0]);
    */
}


/*******************************************************************************/
void programEnd(void){
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7

=======
    //if(_port)
	//_port->stop();
    //delete _port;
>>>>>>> Moved to socketCAN
=======

>>>>>>> changed log
}

/*******************************************************************************/
void processTask_thread(void) {
    //std::cout << "processTask_thread" << std::endl;
    //CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint16_t timer1msPrevious = CO_timer1ms, timerNext_ms = 50;
    boost::posix_time::ptime tick;
    boost::posix_time::time_duration diff;
    while(reset_NMT == CO_RESET_NOT) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
=======
>>>>>>> changed log
        tick = boost::posix_time::microsec_clock::local_time();
        uint16_t timer1msCopy, timer1msDiff;

        timer1msCopy = CO_timer1ms;
        timer1msDiff = timer1msCopy - timer1msPrevious;
        timer1msPrevious = timer1msCopy;
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
        reset_NMT = CO_process(CO, timer1msDiff, &timerNext_ms);
        //printf("timerNext_ms %d\n", timerNext_ms);
        /*#ifdef USE_STORAGE
    CO_EE_process(&CO_EEO);
      #endif*/
        boost::this_thread::sleep(boost::posix_time::milliseconds(timerNext_ms));
        diff = boost::posix_time::microsec_clock::local_time() - tick;
        //std::cout << "processTask_thread: " << diff.total_milliseconds() << " milliseconds" << std::endl;
    }
    DEBUG << "processTask_thread done!";
=======
	tick = boost::posix_time::microsec_clock::local_time();
	uint16_t timer1msCopy, timer1msDiff;

	timer1msCopy = CO_timer1ms;
	timer1msDiff = timer1msCopy - timer1msPrevious;
	timer1msPrevious = timer1msCopy;
    reset_NMT = CO_process(CO, 50, NULL);
	//printf("timerNext_ms %d\n", timerNext_ms);
	/*#ifdef USE_STORAGE
	CO_EE_process(&CO_EEO);
=======
        reset_NMT = CO_process(CO, 50, NULL);
        //printf("timerNext_ms %d\n", timerNext_ms);
        /*#ifdef USE_STORAGE
    CO_EE_process(&CO_EEO);
>>>>>>> changed log
      #endif*/
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        diff = boost::posix_time::microsec_clock::local_time() - tick;
        //std::cout << "processTask_thread: " << diff.total_milliseconds() << " milliseconds" << std::endl;
    }
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
    LOG(DEBUG) << "processTask_thread done!";
>>>>>>> Moved to socketCAN
=======
    BOOST_LOG_TRIVIAL(debug) << "processTask_thread done!";
>>>>>>> changed log
}

/*******************************************************************************/
/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void) {
    //std::cout << "tmrTask_thread" << std::endl;
    //boost::posix_time::ptime tick;
    //boost::posix_time::time_duration diff;
    while(reset_NMT == CO_RESET_NOT) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
=======
>>>>>>> changed log
        //tick = boost::posix_time::microsec_clock::local_time();
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        INCREMENT_1MS(CO_timer1ms);

        /* Lock PDOs and OD */
        CO_LOCK_OD();

        if(CO->CANmodule[0]->CANnormal) {
            bool_t syncWas;

            // Process Sync and read inputs
            syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);

            // Further I/O or nonblocking application code may go here.

            // Write outputs
            CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);

            if (OD_errorRegister > 0) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
                DEBUG << "Check errors - OD_errorRegister: 0x" << std::hex << (int)OD_errorRegister;
=======
                BOOST_LOG_TRIVIAL(debug) << "Check errors - OD_errorRegister: 0x" << std::hex << (int)OD_errorRegister;
>>>>>>> changed log
                //InterEmergSignal();
            }
            //if ()
            // verify timer overflow
            if(0) {
                CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
            }
        }
        /* Unlock */
        CO_UNLOCK_OD();

        //diff = boost::posix_time::microsec_clock::local_time() - tick;
        //std::cout << "The time taken was " << diff.total_milliseconds() << " milliseconds" << std::endl;
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
    }
    DEBUG << "tmrTask_thread done!";
=======
	//tick = boost::posix_time::microsec_clock::local_time();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	INCREMENT_1MS(CO_timer1ms);

	/* Lock PDOs and OD */
	CO_LOCK_OD();

	if(CO->CANmodule[0]->CANnormal) {
	    bool_t syncWas;

	    // Process Sync and read inputs
	    syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);

	    // Further I/O or nonblocking application code may go here.

	    // Write outputs
	    CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);

	    if (OD_errorRegister > 0) {
		LOG(DEBUG) << "Check errors - OD_errorRegister: 0x" << std::hex << (int)OD_errorRegister;
		//InterEmergSignal();
	    }
	    //if ()
	    // verify timer overflow
	    if(0) {
		CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
	    }
	}
	/* Unlock */
	CO_UNLOCK_OD();

	//diff = boost::posix_time::microsec_clock::local_time() - tick;
	//std::cout << "The time taken was " << diff.total_milliseconds() << " milliseconds" << std::endl;
    }
    LOG(DEBUG) << "tmrTask_thread done!";
>>>>>>> Moved to socketCAN
=======
    }
    BOOST_LOG_TRIVIAL(debug) << "tmrTask_thread done!";
>>>>>>> changed log
}

void tmrTask_main(void) {
    //std::cout << "tmrTask_thread" << std::endl;
    //boost::posix_time::ptime tick;
    //boost::posix_time::time_duration diff;
    while(reset_NMT == CO_RESET_NOT && CO_endProgram == 0) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
        /* loop for normal program execution ******************************************/
        int ready;
        struct epoll_event ev;

        ready = epoll_wait(mainline_epoll_fd, &ev, 1, -1);

        if(ready != 1) {
            if(errno != EINTR) {
                CO_error(0x11100000L + errno);
            }
        } else if(taskMain_process(ev.data.fd, &reset_NMT, CO_timer1ms)) {
            uint16_t timer1msDiff;
            static uint16_t tmr1msPrev = 0;

            /* Calculate time difference */
            timer1msDiff = CO_timer1ms - tmr1msPrev;
            tmr1msPrev = CO_timer1ms;

            /* code was processed in the above function. Additional code process below */

            /* Execute optional additional application code */
            //app_programAsync(timer1msDiff);

            //CO_OD_storage_autoSave(&odStorAuto, CO_timer1ms, 60000);
        }

        else {
            /* No file descriptor was processed. */
            CO_error(0x11200000L);
        }
    }
    DEBUG << "tmrTask_main done!";
}

boost::posix_time::ptime count = boost::posix_time::microsec_clock::local_time();


=======
/* loop for normal program execution ******************************************/
	int ready;
	struct epoll_event ev;
=======
        /* loop for normal program execution ******************************************/
        int ready;
        struct epoll_event ev;
>>>>>>> changed log

        ready = epoll_wait(mainline_epoll_fd, &ev, 1, -1);

        if(ready != 1) {
            if(errno != EINTR) {
                CO_error(0x11100000L + errno);
            }
        }             else if(taskMain_process(ev.data.fd, &reset_NMT, CO_timer1ms)) {
            uint16_t timer1msDiff;
            static uint16_t tmr1msPrev = 0;

            /* Calculate time difference */
            timer1msDiff = CO_timer1ms - tmr1msPrev;
            tmr1msPrev = CO_timer1ms;

            /* code was processed in the above function. Additional code process below */

            /* Execute optional additional application code */
            //app_programAsync(timer1msDiff);

            //CO_OD_storage_autoSave(&odStorAuto, CO_timer1ms, 60000);
        }

        else {
            /* No file descriptor was processed. */
            CO_error(0x11200000L);
        }
    }
    BOOST_LOG_TRIVIAL(debug) << "tmrTask_main done!";
}

>>>>>>> Moved to socketCAN
/* Realtime thread for CAN receive and taskTmr ********************************/
static void* rt_thread(void* arg) {

    /* Endless loop */
    while(CO_endProgram == 0) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
        //count = boost::posix_time::microsec_clock::local_time();
        int ready;
        struct epoll_event ev;

        ready = epoll_wait(rt_thread_epoll_fd, &ev, 1, -1);

        if(ready != 1) {
            if(errno != EINTR) {
                CO_error(0x12100000L + errno);
            }
        }

        else if(CANrx_taskTmr_process(ev.data.fd)) {
            int i;

            /* code was processed in the above function. Additional code process below */
            INCREMENT_1MS(CO_timer1ms);

            /* Monitor variables with trace objects */
            CO_time_process(&CO_time);
#if CO_NO_TRACE > 0
            for(i=0; i<OD_traceEnable && i<CO_NO_TRACE; i++) {
                CO_trace_process(CO->trace[i], *CO_time.epochTimeOffsetMs);
            }
#endif

            /* Execute optional additional application code */
            //app_program1ms();

            /* Detect timer large overflow */
            if(OD_performance[ODA_performance_timerCycleMaxTime] > TMR_TASK_OVERFLOW_US && rtPriority > 0 && CO->CANmodule[0]->CANnormal) {
                CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0x22400000L | OD_performance[ODA_performance_timerCycleMaxTime]);
            }
        }

        else {
            /* No file descriptor was processed. */
            CO_error(0x12200000L);
        }
        //boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - count;
        //std::cout << "The time taken was " << diff.total_milliseconds() << " milliseconds" << std::endl;
=======
	int ready;
	struct epoll_event ev;
=======
        int ready;
        struct epoll_event ev;
>>>>>>> changed log

        ready = epoll_wait(rt_thread_epoll_fd, &ev, 1, -1);

        if(ready != 1) {
            if(errno != EINTR) {
                CO_error(0x12100000L + errno);
            }
        }

        else if(CANrx_taskTmr_process(ev.data.fd)) {
            int i;

            /* code was processed in the above function. Additional code process below */
            INCREMENT_1MS(CO_timer1ms);

            /* Monitor variables with trace objects */
            CO_time_process(&CO_time);
#if CO_NO_TRACE > 0
            for(i=0; i<OD_traceEnable && i<CO_NO_TRACE; i++) {
                CO_trace_process(CO->trace[i], *CO_time.epochTimeOffsetMs);
            }
#endif

            /* Execute optional additional application code */
            //app_program1ms();

            /* Detect timer large overflow */
            if(OD_performance[ODA_performance_timerCycleMaxTime] > TMR_TASK_OVERFLOW_US && rtPriority > 0 && CO->CANmodule[0]->CANnormal) {
                CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0x22400000L | OD_performance[ODA_performance_timerCycleMaxTime]);
            }
        }

<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
	else {
	    /* No file descriptor was processed. */
	    CO_error(0x12200000L);
	}
>>>>>>> Moved to socketCAN
=======
        else {
            /* No file descriptor was processed. */
            CO_error(0x12200000L);
        }
>>>>>>> changed log
    }

    return NULL;
}

<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
=======
boost::posix_time::ptime count = boost::posix_time::microsec_clock::local_time();

>>>>>>> Moved to socketCAN
/*
void on_receive_can(const std::string &data) {
    std::vector<std::string> strs;
    //count = boost::posix_time::microsec_clock::local_time();
    boost::split(strs,data,boost::is_any_of(" "));
    #ifdef CO_DEBUG
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    std::cout << "READ:-->" << data << std::endl;
=======
	std::cout << "READ:-->" << data << std::endl;
>>>>>>> Moved to socketCAN
=======
    std::cout << "READ:-->" << data << std::endl;
>>>>>>> changed log
    #endif
    std::cout << "READ:-->" << data << std::endl;
    //LOG(DEBUG) << "READ:-->" << data;
    //std::cout << "READ:-->" << data << std::endl;
    //boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - count;
    //std::cout << "The time taken was " << diff.total_milliseconds() << " milliseconds" << std::endl;
    //std::cout << "READ:-->" << data << std::endl;
    //count = boost::posix_time::microsec_clock::local_time();
    //std::cout << "SIZE split: " << strs.size() << std::endl;
    //strs.erase(strs.end());

    CO_CANrxMsg_t rcvMsg;
    if (strs.size() == 12) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
=======
>>>>>>> changed log
    //strs.at<unsigned short>(0); //CANFormat -> CANStandard = 0, CANExtended = 1
    //strs.at<unsigned short>(1); //CANType -> CANData   = 0, CANRemote = 1
    //for (unsigned char i=0; i<strs.size(); i++) {
    //    std::cout << "data[" << (int)i << "]: " << strs[i] << "size: " << strs[i].size() << std::endl;
    //}
    try {
        rcvMsg.ident = boost::lexical_cast<unsigned int>(strs[2]);
        rcvMsg.DLC = boost::lexical_cast<unsigned short>(strs[3]);
        //std::cout << "id: " << rcvMsg.ident << " -- " << boost::lexical_cast<unsigned int>(strs[2]) << std::endl;
        //std::cout << "length: " << (int)rcvMsg.DLC << " -- " << boost::lexical_cast<unsigned short>(strs[3]) << std::endl;
        for (unsigned char i=0; i<rcvMsg.DLC; i++) {
        //std::cout << "data[" << (int)i << "]: " << strs[i+4] << "size: " << strs[i+4].size() << std::endl;
        rcvMsg.data[i] = boost::lexical_cast<unsigned short>(strs[i+4]);
        //std::cout << "data[" << (int)i << "]: " << (int)rcvMsg.data[i] << " -- " << boost::lexical_cast<unsigned short>(strs[i+4]) << std::endl;
        }
    } catch( boost::bad_lexical_cast const&e ) {
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
        std::cout << "Error: input string was not valid -> " << e.what() << std::endl;
    }

    CO_CANinterrupt_Rx(&rcvMsg);
=======
	//strs.at<unsigned short>(0); //CANFormat -> CANStandard = 0, CANExtended = 1
	//strs.at<unsigned short>(1); //CANType -> CANData   = 0, CANRemote = 1
	//for (unsigned char i=0; i<strs.size(); i++) {
	//    std::cout << "data[" << (int)i << "]: " << strs[i] << "size: " << strs[i].size() << std::endl;
	//}
	try {
	    rcvMsg.ident = boost::lexical_cast<unsigned int>(strs[2]);
	    rcvMsg.DLC = boost::lexical_cast<unsigned short>(strs[3]);
	    //std::cout << "id: " << rcvMsg.ident << " -- " << boost::lexical_cast<unsigned int>(strs[2]) << std::endl;
	    //std::cout << "length: " << (int)rcvMsg.DLC << " -- " << boost::lexical_cast<unsigned short>(strs[3]) << std::endl;
	    for (unsigned char i=0; i<rcvMsg.DLC; i++) {
		//std::cout << "data[" << (int)i << "]: " << strs[i+4] << "size: " << strs[i+4].size() << std::endl;
		rcvMsg.data[i] = boost::lexical_cast<unsigned short>(strs[i+4]);
		//std::cout << "data[" << (int)i << "]: " << (int)rcvMsg.data[i] << " -- " << boost::lexical_cast<unsigned short>(strs[i+4]) << std::endl;
	    }
	} catch( boost::bad_lexical_cast const&e ) {
=======
>>>>>>> changed log
        std::cout << "Error: input string was not valid -> " << e.what() << std::endl;
    }

<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
	CO_CANinterrupt_Rx(&rcvMsg);
>>>>>>> Moved to socketCAN
=======
    CO_CANinterrupt_Rx(&rcvMsg);
>>>>>>> changed log
    } else {
        std::cout << "Error: input string was not valid" << std::endl;
    }
}
*/

/*
void CO_Emergency_Handler() {
    LOG(ERROR) << "CO_Emergency_Handler!!!!!!";
    uint16_t errorCode = *(uint16_t*)CO->em->bufReadPtr;
    uint8_t errorBit = *(uint8_t*)(CO->em->bufReadPtr+3);
    uint32_t infoCode = *(uint32_t*)(CO->em->bufReadPtr+4);

    LOG(DEBUG) << "errorBit: 0x" << std::hex << (int)errorBit << "\t errorCode: 0x" << errorCode << "\t infoCode: 0x" << infoCode;

    switch (errorBit) {
    case CO_EM_HEARTBEAT_CONSUMER:

<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
<<<<<<< 78f8fc24aae6a70107496f0aa989dd1c564f07c7
    break;
    case CO_EM_HB_CONSUMER_REMOTE_RESET:
    CO_errorReset(CO->em, errorBit, infoCode);
    break;
    default:
    break;
=======
	break;
=======
    break;
>>>>>>> changed log
    case CO_EM_HB_CONSUMER_REMOTE_RESET:
    CO_errorReset(CO->em, errorBit, infoCode);
    break;
    default:
<<<<<<< 93a1950d94f34ad8fbaff668d099055a58a68a4e
	break;
>>>>>>> Moved to socketCAN
=======
    break;
>>>>>>> changed log
    }

}*/
