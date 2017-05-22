/**
 * CANopen Emergency consumer.
 *
 * @file        CO_EMconsumer.h
 * @ingroup     CO_EMconsumer
 * @author      Angel Merino-Sastre
 * @copyright   2017 Angel Merino-Sastre
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


#ifndef CO_EM_CONS_H
#define CO_EM_CONS_H


/**
 * @defgroup CO_EMconsumer Emergency consumer
 * @ingroup CO_CANopen
 * @{
 *
 * CANopen Emergency consumer.
 *
 * Emergency consumer monitors Emergency messages from remote nodes. If any
 * monitored node don't send his Heartbeat in specified time, Heartbeat consumer
 * sends emergency message. If all monitored nodes are operational, then
 * variable _allMonitoredOperational_ inside CO_EMconsumer_t is set to true.
 * Monitoring starts after the reception of the first HeartBeat (not bootup).
 *
 * @see  @ref CO_Emergency
 */

/**
 * Size of internal buffer, whwre emergencies are stored after CO_errorReport().
 * Buffer is cleared by CO_EM_process().
 */
#define CO_EMcons_INTERNAL_BUFFER_SIZE      10

/**
 * One monitored node inside CO_EMconsumer_t.
 */
typedef struct{
    uint8_t             errorBit;       /**< Of the remote node */
    uint8_t             errorRegister;     /**< True after reception of the first Heartbeat mesage */
    uint16_t            errorCode;	/**< Time since last heartbeat received */
    uint32_t            infoCode;	/**< Consumer heartbeat time from OD */
}CO_EMconsError_t;

/**
 * One monitored node inside CO_EMconsumer_t.
 */
typedef struct{
    uint8_t             numErrors;       /**< Of the remote node */
    CO_EMconsError_t    errorsBuf[CO_EMcons_INTERNAL_BUFFER_SIZE];     /**< True after reception of the first Heartbeat mesage */
    //uint16_t            timeoutTimer;   /**< Time since last heartbeat received */
    //uint16_t            time;           /**< Consumer heartbeat time from OD */
    uint16_t		nodeID;
    uint8_t             CANrxData[1][8]; /**< 8 data bytes of the received message. */
    bool_t              CANrxNew;       /**< True if new Emergency message received from the CAN bus */
}CO_EMconsNode_t;

/**
 * Emergency consumer object.
 *
 * Object is initilaized by CO_EMconsumer_init(). It contains an array of
 * CO_EMconsNode_t objects.
 */
typedef struct{
    CO_EM_t            *em;             /**< From CO_EMconsumer_init() */
    const uint32_t     *EMconsConfig;     /**< From CO_EMconsumer_init() */
    CO_EMconsNode_t    *monitoredNodes; /**< From CO_EMconsumer_init() */
    uint8_t             numberOfMonitoredNodes; /**< From CO_EMconsumer_init() */
    /** True, if all monitored nodes are NMT operational or no node is
        monitored. Can be read by the application */
    //uint8_t             allMonitoredOperational;
    CO_CANmodule_t     *CANdevRx;       /**< From CO_EMconsumer_init() */
    uint16_t            CANdevRxIdxStart; /**< From CO_EMconsumer_init() */
    void              (*pFunctSignal)(void);/**< From CO_EM_initCallback() or NULL */
}CO_EMconsumer_t;


/**
 * Initialize Heartbeat consumer object.
 *
 * Function must be called in the communication reset section.
 *
 * @param EMcons This object will be initialized.
 * @param em Emergency object.
 * @param SDO SDO server object.
 * @param EMconsTime Pointer to _Consumer Heartbeat Time_ array
 * from Object Dictionary (index 0x1016). Size of array is equal to numberOfMonitoredNodes.
 * @param monitoredNodes Pointer to the externaly defined array of the same size
 * as numberOfMonitoredNodes.
 * @param numberOfMonitoredNodes Total size of the above arrays.
 * @param CANdevRx CAN device for Heartbeat reception.
 * @param CANdevRxIdxStart Starting index of receive buffer in the above CAN device.
 * Number of used indexes is equal to numberOfMonitoredNodes.
 *
 * @return #CO_ReturnError_t CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_EMconsumer_init(CO_EMconsumer_t        *EMcons,
	CO_EM_t                *em,
	CO_SDO_t               *SDO,
	const uint32_t EMconsConfig[],
	CO_EMconsNode_t         monitoredNodes[],
	uint8_t                 numberOfMonitoredNodes,
	CO_CANmodule_t         *CANdevRx,
	uint16_t                CANdevRxIdxStart);


/**
 * Process Heartbeat consumer object.
 *
 * Function must be called cyclically.
 *
 * @param EMcons This object.
 * @param NMTisPreOrOperational True if this node is NMT_PRE_OPERATIONAL or NMT_OPERATIONAL.
 * @param timeDifference_ms Time difference from previous function call in [milliseconds].
 */
void CO_EMconsumer_process(
	CO_EMconsumer_t        *EMcons,
        bool_t                  NMTisPreOrOperational,
        uint16_t                timeDifference_ms);

/**
 * Initialize Emergency consumer callback function.
 *
 * Function initializes optional callback function, which executes after
 * error condition is changed. Function may wake up external task,
 * which processes mainline CANopen functions.
 *
 * @param em This object.
 * @param pFunctSignal Pointer to the callback function. Not called if NULL.
 */
void CO_EMconsumer_initCallback(
	CO_EMconsumer_t               *EMcons,
	void                  (*pFunctSignal)(void));

/** @} */
#endif // CO_EM_CONS_H
