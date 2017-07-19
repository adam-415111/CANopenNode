/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2015 Janez Paternoster
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

#include <mbed.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "CO_driver.h"
#include "CO_Emergency.h"

#ifdef __cplusplus
} // extern "C"
#endif

mbed::CAN *_can;

#ifdef CO_DEBUG
  DigitalOut read_activity(LED2); // CAN read toggle led
  DigitalOut write_activity(LED1);// CAN write toggle led
#endif

//Defined as mbed callbacks does not support parameters
CO_CANmodule_t *_CANmodule;

/******************************************************************************/
void CO_CANsetConfigurationMode(/*mbed::CAN*/int32_t CANbaseAddress){
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */

    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        int32_t                 CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANbaseAddress = CANbaseAddress;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;//(rxSize <= 32U) ? true : false;/* microcontroller dependent */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;
    CANmodule->em = NULL;

    _CANmodule = CANmodule;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].pFunct = NULL;
    }
    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
    }

    _can = (mbed::CAN*)CANmodule->CANbaseAddress;
    _can->attach(&CO_CANinterrupt_Rx, CAN::RxIrq);
    //Disable because reset does not work.
    //_can->attach(&CO_CANinterrupt_busErr, CAN::BeIrq); //CAN::BeIrq for bus error
    //_can->attach(&CO_CANinterrupt_err, CAN::EwIrq); //Error warning
    //_can->attach(&CO_CANinterrupt_do, CAN::DoIrq);// CAN::DoIrq for data overrun
    //_can->attach(&CO_CANinterrupt_pe, CAN::EpIrq);// CAN::EpIrq for error passiv

    /* Configure CAN module registers */


    /* Configure CAN timing */


    /* Configure CAN module hardware filters */
    if(CANmodule->useCANrxFilters){
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received message must match filter */
    }
    else{
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        /* Configure mask 0 so, that all messages with standard identifier are accepted */
    }

    /* configure CAN interrupt registers */

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    /* turn off the module */
}


/******************************************************************************/
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg){
    return (uint16_t) rxMsg->ident;
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (pFunct!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->pFunct = pFunct;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if(rtr){
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){

        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = ((uint32_t)ident & 0x07FFU)
                      | ((uint32_t)(((uint32_t)noOfBytes & 0xFU) << 12U))
                      | ((uint32_t)(rtr ? 0x8000U : 0U));

        buffer->DLC = noOfBytes;

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

//Timer t;
/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;
    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            //don't set error, if bootup message is still on buffers
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND();
    /* if CAN TX buffer is free, copy message to it */
    if(CANmodule->CANtxCount == 0){
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        /* copy message and txRequest */
        //CANMessage canMsg(msg.id, (char *)msg.data, msg.length);
        mbed::CANMessage canMsg;
        canMsg.id = buffer->ident;
        //msg.format = 0; CANStandard 11b
        //msg.type = 0; CANData
        canMsg.len = buffer->DLC;
        for(int i = 0; i < canMsg.len; i++) canMsg.data[i] = buffer->data[i];

        _can = (mbed::CAN*)CANmodule->CANbaseAddress;
        //_can->write(canMsg);
        if (_can->write(canMsg) == 0) {
          printf("NO SEND!");
          CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
          err =  CO_ERROR_TX_OVERFLOW; // TODO
        }
        #ifdef CO_DEBUG
          printf("Send:    [ ID: %11x", canMsg.id);
          printf(" Length: %d", canMsg.len);
          printf(" Data: %2x", canMsg.data[0]);
          printf(" %2x", canMsg.data[1]);
          printf(" %2x", canMsg.data[2]);
          printf(" %2x", canMsg.data[3]);
          printf(" %2x", canMsg.data[4]);
          printf(" %2x", canMsg.data[5]);
          printf(" %2x", canMsg.data[6]);
          printf(" %2x", canMsg.data[7]);
          printf(" Type: %d", canMsg.type);           // 0 = data, 1 = remote
          printf(" Format: %d ]\r\n", canMsg.format);
          /*printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n",
          //printf("%d %d %d %d %2x %2x %2x %2x %2x %2x %2x %2x\r\n",
          //printf("%u %u %u %u %u %u %u %u %u %u %u %u\r\n",
            canMsg.format,canMsg.type,canMsg.id,canMsg.len,
            canMsg.data[0],canMsg.data[1],canMsg.data[2],canMsg.data[3],
            canMsg.data[4],canMsg.data[5],canMsg.data[6],canMsg.data[7]);*/
          //write_activity = !write_activity;             //Blink!
        #endif

    }
    /* if no buffer is free, message will be sent by interrupt */
    else{
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND();
    /*t.stop();
    printf("Sending after %d milliseconds\n", t.read_ms());
    t.reset();
    t.start();*/
    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND();
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND();


    if(tpdoDeleted != 0U){
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
    }
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule){
  //Disable because can.reset() causes the communication to stop without the possibility binging it back
  return;
    uint16_t rxErrors, txErrors, overflow;
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
    uint32_t err;

    /* get error counters from module. Id possible, function may use different way to
     * determine errors. */
    /*rxErrors = CANmodule->txSize;
    txErrors = CANmodule->txSize;
    overflow = CANmodule->txSize;*/
    _can = (mbed::CAN*)CANmodule->CANbaseAddress;
    rxErrors = /*CANmodule->CANbaseAddress*/_can->rderror();
    txErrors = /*CANmodule->CANbaseAddress*/_can->tderror();
    overflow = rxErrors + rxErrors;
    overflow = rxErrors = rxErrors = 0;
    /*printf("rxErrors %d\n", rxErrors);
    printf("txErrors %d\n", txErrors);
    printf("overflow %d\n", overflow);*/
    //printf("em->errorStatusBits[3]: %d\n", em->errorStatusBits[3]);

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;
    //printf("err %d\n", err);
    CANmodule->errOld = 0;
    if(CANmodule->errOld != err) {
        CANmodule->errOld = err;

        if(txErrors >= 256U){                               /* bus off */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else{                                               /* not bus off */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if((rxErrors >= 96U) || (txErrors >= 96U)){     /* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
            }

            if(rxErrors >= 128U){                           /* RX bus passive */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
            }

            if(txErrors >= 128U){                           /* TX bus passive */
                if(!CANmodule->firstCANtxMessage){
                    CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if((rxErrors < 96U) && (txErrors < 96U)){       /* no error */
                CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
            }
        }

        if(overflow != 0U){                                 /* CAN RX bus overflow */
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
        }
    }
    /*printf("em->errorStatusBits[3]: %d\n", em->errorStatusBits[3]);*/
}


/******************************************************************************/
void CO_CANinterrupt(CO_CANmodule_t *CANmodule){

    /* receive interrupt */
    if(1){
        CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
        uint16_t index;             /* index of received message */
        uint32_t rcvMsgIdent;       /* identifier of the received message */
        CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        rcvMsg = 0; /* get message from module here */
        rcvMsgIdent = rcvMsg->ident;
        if(CANmodule->useCANrxFilters){
            /* CAN module filters are used. Message with known 11-bit identifier has */
            /* been received */
            index = 0;  /* get index of the received message here. Or something similar */
            if(index < CANmodule->rxSize){
                buffer = &CANmodule->rxArray[index];
                /* verify also RTR */
                if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                }
            }
        }
        else{
            /* CAN module filters are not used, message with any standard 11-bit identifier */
            /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
            buffer = &CANmodule->rxArray[0];
            for(index = CANmodule->rxSize; index > 0U; index--){
                if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                    break;
                }
                buffer++;
            }
        }

        /* Call specific function, which will process the message */
        if(msgMatched && (buffer != NULL) && (buffer->pFunct != NULL)){
            buffer->pFunct(buffer->object, rcvMsg);
        }

        /* Clear interrupt flag */
    }


    /* transmit interrupt */
    else if(0){
        /* Clear interrupt flag */

        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        if(CANmodule->CANtxCount > 0U){
            uint16_t i;             /* index of transmitting message */

            /* first buffer */
            CO_CANtx_t *buffer = &CANmodule->txArray[0];
            /* search through whole array of pointers to transmit message buffers. */
            for(i = CANmodule->txSize; i > 0U; i--){
                /* if message buffer is full, send it. */
                if(buffer->bufferFull){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;

                    /* Copy message to CAN buffer */
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    /* canSend... */
                    break;                      /* exit for loop */
                }
                buffer++;
            }/* end of for loop */

            /* Clear counter if no more messages */
            if(i == 0U){
                CANmodule->CANtxCount = 0U;
            }
        }
    }
    else{
        /* some other interrupt reason */
    }
}

/******************************************************************************/
void CO_CANinterrupt_Rx()
{
  #ifdef CO_DEBUG
  //printf("RECEIVING: ");
  #endif
  CO_CANrxMsg_t rcvMsg;      /* pointer to received message in CAN module */
  CANMessage canMsg;
  uint16_t index;             /* index of received message */
  uint32_t rcvMsgIdent;       /* identifier of the received message */
  CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
  bool_t msgMatched = false;

  //rcvMsg = 0; /* get message from module here */
  _can = (mbed::CAN*)_CANmodule->CANbaseAddress;
  if (!/*_CANmodule->CANbaseAddress*/_can->read(canMsg)) return;
  #ifdef CO_DEBUG
    printf("Read:    [ ID: %11x", canMsg.id);
    printf(" Length: %d", canMsg.len);
    printf(" Data: %2x", canMsg.data[0]);
    printf(" %2x", canMsg.data[1]);
    printf(" %2x", canMsg.data[2]);
    printf(" %2x", canMsg.data[3]);
    printf(" %2x", canMsg.data[4]);
    printf(" %2x", canMsg.data[5]);
    printf(" %2x", canMsg.data[6]);
    printf(" %2x", canMsg.data[7]);
    printf(" Type: %d", canMsg.type);           // 0 = data, 1 = remote

    //printf(" Format: %d ]\r\n", canMsg.format);
    //printf("%u %u %u %u %u %u %u %u %u %u %u %u\r\n",
    //printf("%d %d %d %d %2x %2x %2x %2x %2x %2x %2x %2x\r\n",
    /*printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n",
      canMsg.format,canMsg.type,canMsg.id,canMsg.len,
      canMsg.data[0],canMsg.data[1],canMsg.data[2],canMsg.data[3],
      canMsg.data[4],canMsg.data[5],canMsg.data[6],canMsg.data[7]);*/
    //read_activity = !read_activity;             //Blink!
  #endif
  rcvMsg.ident = canMsg.id;
  rcvMsg.DLC =canMsg.len;
  memcpy(rcvMsg.data, canMsg.data, canMsg.len);
  rcvMsgIdent = rcvMsg.ident;

  if(_CANmodule->useCANrxFilters) {
    /* CAN module filters are used. Message with known 11-bit identifier has */
    /* been received */
    index = 0;  /* get index of the received message here. Or something similar */
    if(index < _CANmodule->rxSize) {
      buffer = &_CANmodule->rxArray[index];
      /* verify also RTR */
      if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
          msgMatched = true;
      }
    }
  } else {
      /* CAN module filters are not used, message with any standard 11-bit identifier */
      /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
      buffer = &_CANmodule->rxArray[0];
      for(index = _CANmodule->rxSize; index > 0U; index--){
          if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
              msgMatched = true;
              break;
          }
          buffer++;
      }
  }
  //printf("msgMatched: %d\n", msgMatched);
  /* Call specific function, which will process the message */
  if(msgMatched && (buffer != NULL) && (buffer->pFunct != NULL)){
      //printf("msgMatched!\n");
      buffer->pFunct(buffer->object, &rcvMsg);
  }

}

/******************************************************************************/
CO_ReturnError_t CO_mbedCANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {

  //CANMessage canMsg(msg.id, (char *)msg.data, msg.length);
  mbed::CANMessage msg;
  msg.id = buffer->ident;
  //msg.format = 0; CANStandard 11b
  //msg.type = 0; CANData
  msg.len = buffer->DLC;
  for(int i = 0; i < msg.len; i++) msg.data[i] = buffer->data[i];
  _can = (mbed::CAN*)CANmodule->CANbaseAddress;
  if (_can->write(msg) == 1)
    return CO_ERROR_NO;
  else
    return CO_ERROR_TX_OVERFLOW; // TODO

}

/******************************************************************************/
void CO_CANinterrupt_busErr()
{
    printf("CAN BUSERR\n");
    /*for (int i=0;i<3;i++) {
      //statusLed = 1;
      wait_ms(300);
      //statusLed = 0;
      wait_ms(50);
    }
    //statusLed = 0;
    wait_ms(3000);
    NVIC_SystemReset();*/
}

/******************************************************************************/
void CO_CANinterrupt_err()
{
    printf("CAN ERR\n");
    /*for (int i=0;i<5;i++) {
      //statusLed = 1;
      wait_ms(50);
      //statusLed = 0;
      wait_ms(300);
    }
    //statusLed = 0;
    wait_ms(3000);
    NVIC_SystemReset();*/
}

void CO_CANinterrupt_do() {
  printf("CAN::DoIrq\n");
}

void CO_CANinterrupt_pe() {
  printf("CAN::EpIrq\n");
}
