/*
 * CANopen Heartbeat consumer object.
 *
 * @file        CO_EMconsumer.c
 * @ingroup     CO_EMconsumer
 * @author      Janez Paternoster
 * @copyright   2004 - 2013 Janez Paternoster
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


#include "CO_driver.h"
#include "CO_SDO.h"
#include "CO_Emergency.h"
//#include "CO_NMT_Heartbeat.h"
#include "CO_EMconsumer.h"
#include "CANopen.h"

/*
 * Read received message from CAN module.
 *
 * Function will be called (by CAN receive interrupt) every time, when CAN
 * message with correct identifier will be received. For more information and
 * description of parameters see file CO_driver.h.
 */
static void CO_EMcons_receive(void *object, const CO_CANrxMsg_t *msg);
static void CO_EMcons_receive(void *object, const CO_CANrxMsg_t *msg) {
    CO_EMconsNode_t *EMconsNode;

    EMconsNode = (CO_EMconsNode_t*) object; /* this is the correct pointer type of the first argument */

    /* verify message length */
    if(msg->DLC == 8){
        /* copy data and set 'new message' flag. */
	EMconsNode->CANrxData[0][0] = msg->data[0];
	EMconsNode->CANrxData[0][1] = msg->data[1];
	EMconsNode->CANrxData[0][2] = msg->data[2];
	EMconsNode->CANrxData[0][3] = msg->data[3];
	EMconsNode->CANrxData[0][4] = msg->data[4];
	EMconsNode->CANrxData[0][5] = msg->data[5];
	EMconsNode->CANrxData[0][6] = msg->data[6];
	EMconsNode->CANrxData[0][7] = msg->data[7];
	EMconsNode->CANrxNew = true;
    }
}


/*
 * Configure one monitored node.
 */
static void CO_EMcons_monitoredNodeConfig(
	CO_EMconsumer_t        *EMcons,
        uint8_t                 idx,
	uint32_t                EMconsConfig)
{
    uint16_t COB_ID;
    //uint16_t NodeID = 0;
    CO_EMconsNode_t *monitoredNode;

    if(idx >= EMcons->numberOfMonitoredNodes) return;

    //NodeID = (uint16_t)((EMconsTime>>16)&0xFF);
    //NodeID = (uint16_t)EMconsConfig;
    monitoredNode = &EMcons->monitoredNodes[idx];
    //monitoredNode->time = (uint16_t)EMconsTime;
    monitoredNode->nodeID =(uint16_t)EMconsConfig;
    monitoredNode->numErrors = 0;
    //monitoredNode->monStarted = false;

    /* is channel used */
    if(monitoredNode->nodeID){
	COB_ID = monitoredNode->nodeID + CO_CAN_ID_EMERGENCY;
    }
    else{
        COB_ID = 0;
	//monitoredNode->time = 0;
    }

    /* configure Heartbeat consumer CAN reception */
    CO_CANrxBufferInit(
	    EMcons->CANdevRx,
	    EMcons->CANdevRxIdxStart + idx,
            COB_ID,
            0x7FF,
            0,
	    (void*)&EMcons->monitoredNodes[idx],
	    CO_EMcons_receive);
}


/*
 * OD function for accessing _COB-ID EMCY consumer_ (index 0x1028) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1028(CO_ODF_arg_t *ODF_arg);
static CO_SDO_abortCode_t CO_ODF_1028(CO_ODF_arg_t *ODF_arg){
    CO_EMconsumer_t *EMcons;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    EMcons = (CO_EMconsumer_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading){
        uint8_t NodeID;
	uint16_t EMconsTime;

	NodeID = value & 0xFFFFU;
	//EMconsTime = value & 0xFFFFU;

        if((value & 0xFF800000U) != 0){
            ret = CO_SDO_AB_PRAM_INCOMPAT;
        }
	else if((NodeID != 0)){
            uint8_t i;
            /* there must not be more entries with same index and time different than zero */
	    for(i = 0U; i<EMcons->numberOfMonitoredNodes; i++){
		uint32_t objectCopy = EMcons->EMconsConfig[i];
		uint8_t NodeIDObj = objectCopy & 0xFFFFU;
		//uint16_t EMconsTimeObj = objectCopy & 0xFFFFU;
		if(((ODF_arg->subIndex-1U) != i) && (NodeID == NodeIDObj)){
                    ret = CO_SDO_AB_PRAM_INCOMPAT;
                }
            }
        }
        else{
            ret = CO_SDO_AB_NONE;
        }

        /* Configure */
        if(ret == CO_SDO_AB_NONE){
	    CO_EMcons_monitoredNodeConfig(EMcons, ODF_arg->subIndex-1U, value);
        }
    }

    return ret;
}


/******************************************************************************/
CO_ReturnError_t CO_EMconsumer_init(
	CO_EMconsumer_t        *EMcons,
        CO_EM_t                *em,
        CO_SDO_t               *SDO,
	const uint32_t          EMconsConfig[],
	CO_EMconsNode_t         monitoredNodes[],
        uint8_t                 numberOfMonitoredNodes,
        CO_CANmodule_t         *CANdevRx,
        uint16_t                CANdevRxIdxStart)
{
    uint8_t i;

    /* verify arguments */
    if(EMcons==NULL || em==NULL || SDO==NULL || EMconsConfig==NULL ||
        monitoredNodes==NULL || CANdevRx==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    EMcons->em = em;
    EMcons->EMconsConfig = EMconsConfig;
    EMcons->monitoredNodes = monitoredNodes;
    EMcons->numberOfMonitoredNodes = numberOfMonitoredNodes;
    //EMcons->allMonitoredOperational = 0;
    EMcons->CANdevRx = CANdevRx;
    EMcons->CANdevRxIdxStart = CANdevRxIdxStart;

    for(i=0; i<EMcons->numberOfMonitoredNodes; i++)
	CO_EMcons_monitoredNodeConfig(EMcons, i, EMcons->EMconsConfig[i]);

    /* Configure Object dictionary entry at index 0x1016 */
    CO_OD_configure(SDO, OD_H1028_EMCY_CONSUMER, CO_ODF_1028, (void*)EMcons, 0, 0);

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_EMconsumer_process(
	CO_EMconsumer_t        *EMcons,
        bool_t                  NMTisPreOrOperational,
        uint16_t                timeDifference_ms)
{
    //printf ("CO_EMconsumer_process\n");
    uint8_t i;
    //uint8_t AllMonitoredOperationalCopy;
    CO_EMconsNode_t *monitoredNode;

    //AllMonitoredOperationalCopy = 5;
    monitoredNode = &EMcons->monitoredNodes[0];

    if(NMTisPreOrOperational) {
	for(i=0; i<EMcons->numberOfMonitoredNodes; i++) {
	    //printf ("monitoredNode i: %d\t", i);
	    if(monitoredNode->nodeID) {/* is node monitored */
		//printf ("monitoredNode i: %d\t", i);
                /* Verify if new Consumer Heartbeat message received */
		if(monitoredNode->CANrxNew) {
		    //printf ("monitoredNode->CANrxNew\n");
		    uint16_t errorCode = *(uint16_t*)monitoredNode->CANrxData[0];
		    uint8_t errorRegister = monitoredNode->CANrxData[0][2];
		    uint8_t errorBit = monitoredNode->CANrxData[0][3];
		    uint32_t infoCode = (uint32_t)monitoredNode->CANrxData[0][4];
		    if (errorCode != 0) { //Error code present, new error report
			CO_EMconsError_t *errorObj = &monitoredNode->errorsBuf[monitoredNode->numErrors];
			errorObj->errorCode = errorCode;
			errorObj->errorRegister = errorRegister;
			errorObj->errorBit = errorBit;
			errorObj->infoCode = infoCode;
			monitoredNode->numErrors++;
			//TODO execute errorHandlers
			if(EMcons->pFunctSignal != NULL) {
			    EMcons->pFunctSignal();
			}
		    } else { // No error code, error reset
			for (unsigned char i = 0; i < monitoredNode->numErrors; i++) {
			    if (errorBit == monitoredNode->errorsBuf[i].errorBit &&
				    infoCode == monitoredNode->errorsBuf[i].infoCode)
			    {
				for (unsigned char j = i+1; j < monitoredNode->numErrors; j++) {
				    monitoredNode->errorsBuf[j-1] = monitoredNode->errorsBuf[j];
				}
				monitoredNode->numErrors--;
				break;
			    }
			}
			//TODO execute errorHandlers clear
		    }
                    monitoredNode->CANrxNew = false;
                }
		monitoredNode++;
	    }
	}
    } else { /* not in (pre)operational state */
	/*for(i=0; i<EMcons->numberOfMonitoredNodes; i++){
	    monitoredNode->NMTstate = 0;
            monitoredNode->CANrxNew = false;
	    monitoredNode->monStarted = false;
            monitoredNode++;
        }
	AllMonitoredOperationalCopy = 0;*/
    }
    //EMcons->allMonitoredOperational = AllMonitoredOperationalCopy;
}

/******************************************************************************/
void CO_EMconsumer_initCallback(
	CO_EMconsumer_t		*EMcons,
	void                  (*pFunctSignal)(void))
{
    if(EMcons != NULL){
	EMcons->pFunctSignal = pFunctSignal;
    }
}
