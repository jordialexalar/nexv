/*
 * can_power_member.c
 *
 *  Created on: 2 oct. 2018
 *      Author: josepmaria.fernandez
 */

#include "main.h"
#include "can_power_member.h"     // Header file (needed)
//#include "DSP2833x_Device.h"      // Adapt to your needs
//External function needed: canTransmitStdMsg(&canFrame) -> This function must be implemented by the user and is platform dependent

//NodeParams thisNodeParams;

__u16 own_node = 0x04;
__u16 supervisor_node = 0x03 ;
__u16 networkIsCriticalFlag = 0;
__u16 masterHasErrorsFlag = 0;
__u16 nodesHaveErrorsFlag = 0;

EventWord eventBuffer[MAXIMUM_ERRORS_LISTED];
EventWord eventBufferM[MAXIMUM_ERRORS_LISTED];

int deltaLastReceivedMessage = 0;
int deltaErrorComm = 0;

unsigned int publishLowLatencyMeasures(const float outputVoltage, const float availableVoltage, const float outputCurrent, const float availableCurrent)
{
    struct can_frame canFrame;
    if (own_node == supervisor_node + 1){
    	mensajeASuper.outputVoltage = outputVoltage;
    	mensajeASuper.availableVoltage = availableVoltage;
    	mensajeASuper.outputCurrent = outputCurrent;
    	mensajeASuper.availableCurrent = availableCurrent;

    	mensajeASuper.mensaje_rec[PUBLISH_LOW_LATENCY] = 1;

    }

   // else{
        canFrame.can_id = 0x040 + own_node;
        canFrame.data[0] = ((int)(outputVoltage*10.0)) >> 8;
        canFrame.data[1] = ((int)(outputVoltage*10.0)) & 0xFF;
        canFrame.data[2] = ((int)(availableVoltage*10.0)) >> 8;
        canFrame.data[3] = ((int)(availableVoltage*10.0)) & 0xFF;
        canFrame.data[4] = ((int)(outputCurrent*10.0)) >> 8;
        canFrame.data[5] = ((int)(outputCurrent*10.0)) & 0xFF;
        canFrame.data[6] = ((int)(availableCurrent*10.0)) >> 8;
        canFrame.data[7] = ((int)(availableCurrent*10.0)) & 0xFF;
        canFrame.can_dlc = 8;
  //  }
   envia_siempre = 1;
   return canTransmitStdMsg(&canFrame);
}


void getLowLatencyMeasures(struct can_frame *canFrame, float *outputVoltage, float *availableVoltage, float *outputCurrent, float *availableCurrent)
{
    if ((canFrame->can_id & 0x040) != 0x040 ||  canFrame->can_dlc != 8) return;
    *outputVoltage      = CAN_2BYTES_TO_FLOAT(canFrame->data[0],canFrame->data[1]) * 0.1;
    *availableVoltage   = CAN_2BYTES_TO_FLOAT(canFrame->data[2],canFrame->data[3]) * 0.1;
    *outputCurrent      = CAN_2BYTES_TO_FLOAT(canFrame->data[4],canFrame->data[5]) * 0.1;
    *availableCurrent   = CAN_2BYTES_TO_FLOAT(canFrame->data[6],canFrame->data[7]) * 0.1;
    return;
}

unsigned int publishState(const __u8 stateNumber, const Output_Config currentConfig, const __u8 contactorState, const __u8 listenedNode, const float maxVoltageRequest, const float maxCurrentRequest)
{
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
    	mensajeASuper.stateNumber = stateNumber;

    	mensajeASuper.currentConfiguration = currentConfig;
    	mensajeASuper.socketContactorStatus = contactorState;
    	mensajeASuper.followedSocket = listenedNode;
    	mensajeASuper.maxVoltageRequest = maxVoltageRequest;
    	mensajeASuper.maxCurrentRequest = maxCurrentRequest;

    	mensajeASuper.mensaje_rec[PUBLISH_STATE] = 1;

    }
  /*  else{ */
        canFrame.can_id = 0x0C0 + own_node;
        canFrame.data[0] = stateNumber ;
        canFrame.data[1] = ((__u8)(currentConfig)) ;
        canFrame.data[2] = listenedNode;
        canFrame.data[3] = contactorState;
        canFrame.data[4] = ((int)(maxVoltageRequest*10.0)) >> 8;
        canFrame.data[5] = ((int)(maxVoltageRequest*10.0)) & 0xFF;
        canFrame.data[6] = ((int)(maxCurrentRequest*10.0)) >> 8;
        canFrame.data[7] = ((int)(maxCurrentRequest*10.0)) & 0xFF;
        canFrame.can_dlc = 8;

  /*  } */

        envia_siempre = 1;

    return canTransmitStdMsg(&canFrame);
}

void getState (struct can_frame *canFrame, __u8 *stateNumber, Output_Config *currentConfig, __u8 *contactorState, __u8 *listenedNode, float *maxVoltageRequest, float *maxCurrentRequest)
{
    if ((canFrame->can_id & 0x0C0) != 0x0C0 ||  canFrame->can_dlc != 8) return;
    *stateNumber        = canFrame->data[0] ;
    *currentConfig      = (Output_Config)canFrame->data[1];
    *listenedNode       = canFrame->data[2] ;
    *contactorState     = canFrame->data[3] ;
    *maxVoltageRequest  = CAN_2BYTES_TO_FLOAT(canFrame->data[4],canFrame->data[5]) * 0.1;
    *maxCurrentRequest  = CAN_2BYTES_TO_FLOAT(canFrame->data[6],canFrame->data[7]) * 0.1;
    return;
}

void getStateSIMUL (__u8 *stateNumber, float *maxVoltageRequest, float *maxCurrentRequest)
{

    *stateNumber        = 2 ;

    *maxVoltageRequest  = 3000 * 0.1;
    *maxCurrentRequest  = 500 * 0.1;
    return;
}


void getStateSIMULNodes (float *maxVoltageRequest, float *maxCurrentRequest)
{



    *maxVoltageRequest  = 6000 * 0.1;
    *maxCurrentRequest  = 900 * 0.1;
    return;
}

unsigned int publishEvent(EventWord *eventList){
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
    	mensajeASuper.eventListMessage.eventLocation.hardware= eventList->eventLocation.hardware;
    	mensajeASuper.eventListMessage.eventLocation.subsystem = eventList->eventLocation.subsystem;
    	mensajeASuper.eventListMessage.eventLocation.subsubsystem = eventList->eventLocation.subsubsystem;        
    	mensajeASuper.eventListMessage.eventLocation.severity = eventList->eventLocation.severity;
    	mensajeASuper.eventListMessage.eventLocation.eventCode = eventList->eventLocation.eventCode;
    	mensajeASuper.eventListMessage.epochTime = eventList->epochTime;
    	mensajeASuper.eventListMessage.uniqueIdentifier = eventList->uniqueIdentifier;

    	mensajeASuper.mensaje_rec[PUBLISH_EVENT] = 1;


    }

    if (own_node == supervisor_node){
    	mensajeDeSuper.eventListMessage.eventLocation.hardware= eventList->eventLocation.hardware;
    	mensajeDeSuper.eventListMessage.eventLocation.subsystem = eventList->eventLocation.subsystem;
    	mensajeDeSuper.eventListMessage.eventLocation.subsubsystem = eventList->eventLocation.subsubsystem;
    	mensajeDeSuper.eventListMessage.eventLocation.severity = eventList->eventLocation.severity;
    	mensajeDeSuper.eventListMessage.eventLocation.eventCode = eventList->eventLocation.eventCode;
    	mensajeDeSuper.eventListMessage.epochTime = eventList->epochTime;
    	mensajeDeSuper.eventListMessage.uniqueIdentifier = eventList->uniqueIdentifier;

    	mensajeDeSuper.mensaje_rec[PUBLISH_EVENT] = 1;


    }

 /*   else{ */
        canFrame.can_id = 0x140 + own_node;
        canFrame.data[0] = ((int)(eventList->eventLocation.hardware)) << 4;
        canFrame.data[0] |= ((int)(eventList->eventLocation.subsystem));
        canFrame.data[1] = ((int)(eventList->eventLocation.subsubsystem)) << 4;
        canFrame.data[1] |= ((int)(eventList->eventLocation.severity));

        canFrame.data[2] = ((int)(eventList->eventLocation.eventCode)) ;

        canFrame.data[3] = ((int)(eventList->epochTime >> 24)) & 0xFF;
        canFrame.data[4] = ((int)(eventList->epochTime >> 16)) & 0xFF;
        canFrame.data[5] = ((int)(eventList->epochTime >> 8)) & 0xFF;
        canFrame.data[6] = ((int)(eventList->epochTime)) & 0xFF;

        canFrame.data[7] = ((int)(eventList->uniqueIdentifier)) & 0xFF;
        canFrame.can_dlc = 8;
  //  }

        envia_siempre = 1;
    return canTransmitStdMsg(&canFrame);
}

void getEvent(const struct can_frame *canFrame, EventWord *eventList){
    if ((canFrame->can_id & 0x140) != 0x140 ||  canFrame->can_dlc != 8) return;

    eventList->eventLocation.hardware = (canFrame->data[0] & 0xF0)>>4;
    eventList->eventLocation.subsystem = (canFrame->data[0] & 0x0F);
    eventList->eventLocation.subsubsystem = (canFrame->data[1] & 0xF0)>>4;

    switch(canFrame->data[1] & 0x0F)
    {
        case 0: eventList->eventLocation.severity = SEVERITY_TRACE; break;
        case 1: eventList->eventLocation.severity = SEVERITY_DEBUG; break;
        case 2: eventList->eventLocation.severity = SEVERITY_INFO; break;
        case 3: eventList->eventLocation.severity = SEVERITY_WARNING; break;
        case 4: eventList->eventLocation.severity = SEVERITY_ERROR; break;
        case 5: eventList->eventLocation.severity = SEVERITY_FATAL; break;
    }

    eventList->eventLocation.eventCode = canFrame->data[2];
    eventList->epochTime = (((__u32)canFrame->data[3])<<24) + (((__u32)canFrame->data[4])<<16) +\
            (((__u32)canFrame->data[5])<<8) + (((__u32)canFrame->data[6]));
    eventList->uniqueIdentifier = canFrame->data[7];
}


void getEventMessage(EventWord *eventList){

    if (own_node == supervisor_node){
        eventList->eventLocation.hardware = mensajeASuper.eventListMessage.eventLocation.hardware;
    	eventList->eventLocation.subsystem = mensajeASuper.eventListMessage.eventLocation.subsystem;
    	eventList->eventLocation.subsubsystem = mensajeASuper.eventListMessage.eventLocation.subsubsystem;        
    	eventList->eventLocation.severity = mensajeASuper.eventListMessage.eventLocation.severity;
    	eventList->eventLocation.eventCode = mensajeASuper.eventListMessage.eventLocation.eventCode;
    	eventList->epochTime = mensajeASuper.eventListMessage.epochTime;
    	eventList->uniqueIdentifier = mensajeASuper.eventListMessage.uniqueIdentifier;
    }
    if (own_node == supervisor_node + 1){
        eventList->eventLocation.hardware = mensajeDeSuper.eventListMessage.eventLocation.hardware;
    	eventList->eventLocation.subsystem = mensajeDeSuper.eventListMessage.eventLocation.subsystem;
    	eventList->eventLocation.subsubsystem = mensajeDeSuper.eventListMessage.eventLocation.subsubsystem;
    	eventList->eventLocation.severity = mensajeDeSuper.eventListMessage.eventLocation.severity;
    	eventList->eventLocation.eventCode = mensajeDeSuper.eventListMessage.eventLocation.eventCode;
    	eventList->epochTime = mensajeDeSuper.eventListMessage.epochTime;
    	eventList->uniqueIdentifier = mensajeDeSuper.eventListMessage.uniqueIdentifier;
    }

}

unsigned int publishTemperatures(const float temperature1, const float temperature2, const float temperature3,\
                                 const float temperature4, const float humidity1, const unsigned char tacho1,
                                 const unsigned char tacho2){
    struct can_frame canFrame;
    if (own_node == supervisor_node + 1){
    	mensajeASuper.temperature1 = temperature1;
    	mensajeASuper.temperature2 = temperature2;
    	mensajeASuper.temperature3 = temperature3;
    	mensajeASuper.temperature4 = temperature4;
    	mensajeASuper.humidity1 = humidity1;
    	mensajeASuper.tacho1 = tacho1;
    	mensajeASuper.tacho2 = tacho2;

    	mensajeASuper.mensaje_rec[PUBLISH_TEMPERATURES] = 1;     }
//    else{
        canFrame.can_id = 0x1C0 + own_node;
        canFrame.data[0] = ((int)(temperature1)) & 0xFF;
        canFrame.data[1] = ((int)(temperature2)) & 0xFF;
        canFrame.data[2] = ((int)(temperature3)) & 0xFF;
        canFrame.data[3] = ((int)(temperature4)) & 0xFF;
        canFrame.data[4] = ((int)(humidity1)) & 0xFF;
        canFrame.data[5] = tacho1;
        canFrame.data[6] = tacho2;
        canFrame.data[7] = 0;
        canFrame.can_dlc = 8;
 //   }
//    sendCanDelta(&canFrame);
        envia_siempre = 1;
    return canTransmitStdMsg(&canFrame);
}

#define CAN_1BYTE_TO_FLOAT(A) ((A & 0x80) > 0)?((float)( (~A & 0xFF)+1) )*-1.0:(float)(A) //Check if it is negative

void getTemperatures(const struct can_frame *canFrame, float *temperature1, float *temperature2, float *temperature3, \
                     float *temperature4, float *humidity1, unsigned char *tacho1, unsigned char *tacho2){
    *temperature1 = CAN_1BYTE_TO_FLOAT(canFrame->data[0]);
    *temperature2 = CAN_1BYTE_TO_FLOAT(canFrame->data[1]);
    *temperature3 = CAN_1BYTE_TO_FLOAT(canFrame->data[2]);
    *temperature4 = CAN_1BYTE_TO_FLOAT(canFrame->data[3]);
    *humidity1 = CAN_1BYTE_TO_FLOAT(canFrame->data[4]);
    *tacho1 = canFrame->data[5];
    *tacho2 = canFrame->data[6];
}

unsigned int publishActiveInductive (const __u32 active, const __u32 inductive){
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
    	mensajeASuper.active = active;
    	mensajeASuper.inductive = inductive;

    	mensajeASuper.mensaje_rec[PUBLISH_ACTIV] = 1;     }
 //   else{
        canFrame.can_id = 0x2C0 + own_node;
        canFrame.data[0] = (active >> 24) & 0xFF;
        canFrame.data[1] = (active >> 16) & 0xFF;
        canFrame.data[2] = (active >> 8) & 0xFF;
        canFrame.data[3] = (active) & 0xFF;
        canFrame.data[4] = (inductive >> 24) & 0xFF;
        canFrame.data[5] = (inductive >> 16) & 0xFF;
        canFrame.data[6] = (inductive >> 8) & 0xFF;
        canFrame.data[7] = (inductive) & 0xFF;
        canFrame.can_dlc = 8;
//    }

        envia_siempre = 1;
    return canTransmitStdMsg(&canFrame);
}


unsigned int publishCapacitiveInstantaneousPower (const __u32 capacitive, const __u32 instantaneousPower){
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
     	mensajeASuper.capacitive = capacitive;
     	mensajeASuper.instantaneousPower = instantaneousPower;

     	mensajeASuper.mensaje_rec[PUBLISH_CAPACITIVE] = 1;
    }
     else{
    	    canFrame.can_id = 0x340 + own_node;
    	    canFrame.data[0] = (capacitive >> 24) & 0xFF;
    	    canFrame.data[1] = (capacitive >> 16) & 0xFF;
    	    canFrame.data[2] = (capacitive >> 8) & 0xFF;
    	    canFrame.data[3] = (capacitive) & 0xFF;
    	    canFrame.data[4] = (instantaneousPower >> 24) & 0xFF;
    	    canFrame.data[5] = (instantaneousPower >> 16) & 0xFF;
    	    canFrame.data[6] = (instantaneousPower >> 8) & 0xFF;
    	    canFrame.data[7] = (instantaneousPower) & 0xFF;
    	    canFrame.can_dlc = 8;
     }


    return canTransmitStdMsg(&canFrame);
}


unsigned int publishCurrentGlobalCurrentPh1 (const __u32 currentGlobal, const __u32 currentPh1){
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
     	mensajeASuper.currentGlobal = currentGlobal;
     	mensajeASuper.currentPh1 = currentPh1;

     	mensajeASuper.mensaje_rec[PUBLISH_CURRENT_PH1] = 1;
    }
     else{
    	    canFrame.can_id = 0x3C0 + own_node;
    	    canFrame.data[0] = (currentGlobal >> 24) & 0xFF;
    	    canFrame.data[1] = (currentGlobal >> 16) & 0xFF;
    	    canFrame.data[2] = (currentGlobal >> 8) & 0xFF;
    	    canFrame.data[3] = (currentGlobal) & 0xFF;
    	    canFrame.data[4] = (currentPh1 >> 24) & 0xFF;
    	    canFrame.data[5] = (currentPh1 >> 16) & 0xFF;
    	    canFrame.data[6] = (currentPh1 >> 8) & 0xFF;
    	    canFrame.data[7] = (currentPh1) & 0xFF;
    	    canFrame.can_dlc = 8;
     }



    return canTransmitStdMsg(&canFrame);
}


unsigned int publishCurrentPh2CurrentPh3 (const __u32 currentPh2, const __u32 currentPh3){
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
     	mensajeASuper.currentPh2 = currentPh2;
     	mensajeASuper.currentPh3 = currentPh3;

     	mensajeASuper.mensaje_rec[PUBLISH_CURRENT_PH2] = 1;
    }
     else{
    	    canFrame.can_id = 0x440 + own_node;
    	    canFrame.data[0] = (currentPh2 >> 24) & 0xFF;
    	    canFrame.data[1] = (currentPh2 >> 16) & 0xFF;
    	    canFrame.data[2] = (currentPh2 >> 8) & 0xFF;
    	    canFrame.data[3] = (currentPh2) & 0xFF;
    	    canFrame.data[4] = (currentPh3 >> 24) & 0xFF;
    	    canFrame.data[5] = (currentPh3 >> 16) & 0xFF;
    	    canFrame.data[6] = (currentPh3 >> 8) & 0xFF;
    	    canFrame.data[7] = (currentPh3) & 0xFF;
    	    canFrame.can_dlc = 8;
     }

    return canTransmitStdMsg(&canFrame);
}


unsigned int publishVoltageGlobalVoltagePh1 (const __u32 voltageGlobal, const __u32 voltagePh1){
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
     	mensajeASuper.voltageGlobal = voltageGlobal;
     	mensajeASuper.voltagePh1 = voltagePh1;

     	mensajeASuper.mensaje_rec[PUBLISH_VOLTAGE_PH1] = 1;
    }
     else{

		canFrame.can_id = 0x4C0 + own_node;
		canFrame.data[0] = (voltageGlobal >> 24) & 0xFF;
		canFrame.data[1] = (voltageGlobal >> 16) & 0xFF;
		canFrame.data[2] = (voltageGlobal >> 8) & 0xFF;
		canFrame.data[3] = (voltageGlobal) & 0xFF;
		canFrame.data[4] = (voltagePh1 >> 24) & 0xFF;
		canFrame.data[5] = (voltagePh1 >> 16) & 0xFF;
		canFrame.data[6] = (voltagePh1 >> 8) & 0xFF;
		canFrame.data[7] = (voltagePh1) & 0xFF;
		canFrame.can_dlc = 8;
     }



    return canTransmitStdMsg(&canFrame);
}


unsigned int publishVoltagePh2VoltagePh3 (const __u32 voltagePh2, const __u32 voltagePh3){
    struct can_frame canFrame;

    if (own_node == supervisor_node + 1){
     	mensajeASuper.voltagePh2 = voltagePh2;
     	mensajeASuper.voltagePh3 = voltagePh3;

     	mensajeASuper.mensaje_rec[PUBLISH_VOLTAGE_PH2] = 1;
    }
     else{
    	    canFrame.can_id = 0x540 + own_node;
    	    canFrame.data[0] = (voltagePh2 >> 24) & 0xFF;
    	    canFrame.data[1] = (voltagePh2 >> 16) & 0xFF;
    	    canFrame.data[2] = (voltagePh2 >> 8) & 0xFF;
    	    canFrame.data[3] = (voltagePh2) & 0xFF;
    	    canFrame.data[4] = (voltagePh3 >> 24) & 0xFF;
    	    canFrame.data[5] = (voltagePh3 >> 16) & 0xFF;
    	    canFrame.data[6] = (voltagePh3 >> 8) & 0xFF;
    	    canFrame.data[7] = (voltagePh3) & 0xFF;
    	    canFrame.can_dlc = 8;
     }



    return canTransmitStdMsg(&canFrame);
}


void getActiveInductive (const struct can_frame *canFrame, __u32 *active, __u32 *inductive )
{
  if ((canFrame->can_id & 0x2C0) != 0x2C0 ||  canFrame->can_dlc != 8) return;
  *active = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
  *inductive = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}


void getCapacitiveInstantaneousPower (const struct can_frame *canFrame, __u32 *capacitive, __u32 *instantaneousPower)
{
  if ((canFrame->can_id & 0x340) != 0x340 ||  canFrame->can_dlc != 8) return;
  *capacitive = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
  *instantaneousPower = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}

void getCurrentGlobalCurrentPh1 (const struct can_frame *canFrame, __u32 *currentGlobal, __u32 *currentPh1)
{
  if ((canFrame->can_id & 0x3C0) != 0x3C0 ||  canFrame->can_dlc != 8) return;
  *currentGlobal = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
  *currentPh1 = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}

void getCurrentPh2CurrentPh3 (const struct can_frame *canFrame, __u32 *currentPh2, __u32 *currentPh3)
{
  if ((canFrame->can_id & 0x440) != 0x440 ||  canFrame->can_dlc != 8) return;
  *currentPh2 = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
  *currentPh3 = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}

void getVoltageGlobalVoltagePh1 (const struct can_frame *canFrame, __u32 *voltageGlobal, __u32 *voltagePh1)
{
  if ((canFrame->can_id & 0x4C0) != 0x4C0 ||  canFrame->can_dlc != 8) return;
  *voltageGlobal = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
  *voltagePh1 = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}

void getVoltagePh2VoltagePh3 (const struct can_frame *canFrame, __u32 *voltagePh2, __u32 *voltagePh3)
{
  if ((canFrame->can_id & 0x540) != 0x540 ||  canFrame->can_dlc != 8) return;
    *voltagePh2 = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
    *voltagePh3 = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}



/*
void getActiveEnergy (const struct can_frame *canFrame, __u32 *energyDc, __u32 *energyAc ){
    if ((canFrame->can_id & 0x240) != 0x240 ||  canFrame->can_dlc != 8) return;
    *energyDc = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
    *energyAc = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}

void getReactiveEnergy  (const struct can_frame *canFrame, __u32 *energyAcInductive, __u32 *energyAcCapacitive ){
    if ((canFrame->can_id & 0x2C0) != 0x2C0 ||  canFrame->can_dlc != 8) return;
    *energyAcInductive = CAN_4BYTES_TO_UINT32(canFrame->data[0],canFrame->data[1],canFrame->data[2],canFrame->data[3]);
    *energyAcCapacitive = CAN_4BYTES_TO_UINT32(canFrame->data[4],canFrame->data[5],canFrame->data[6],canFrame->data[7]);
}
*/



unsigned int publishChargeCommand(const Charge_Command command, const float voltageSetPoint, const float currentSetPoint, const __u16 contactorRequest){
    struct can_frame canFrame;

    canFrame.can_id = 0x080 + own_node;
    canFrame.data[0] = 0;
    canFrame.data[1] = command & 0xFF;
    canFrame.data[2] = ((int)(voltageSetPoint*10.0)) >> 8;
    canFrame.data[3] = ((int)(voltageSetPoint*10.0)) & 0xFF;
    canFrame.data[4] = ((int)(currentSetPoint*10.0)) >> 8;
    canFrame.data[5] = ((int)(currentSetPoint*10.0)) & 0xFF;
    canFrame.data[6] = 0;
    canFrame.data[7] = contactorRequest & 0xFF;
    canFrame.can_dlc = 8;

    return canTransmitStdMsg(&canFrame);
}

void getChargeCommand(const struct can_frame *canFrame, Charge_Command *command, float *voltageSetPoint, float *currentSetPoint, __u16 *contactorRequest){
    if ((canFrame->can_id & 0x080) != 0x080 || canFrame->can_dlc != 8) return;

    switch(CAN_2BYTES_TO_UINT16(canFrame->data[0],canFrame->data[1])){
        case 0: *command = DISCONNECTION_REQUEST; break;
        case 1: *command = WAIT_REQUEST; break;
        case 2: *command = OPERATION_REQUEST; break;
        case 3: *command = STOP_REQUEST; break;
        case 4: *command = REARM_REQUEST; break;
        default: break;
    }

    *voltageSetPoint = CAN_2BYTES_TO_FLOAT(canFrame->data[2],canFrame->data[3]) * 0.1;
    *currentSetPoint = CAN_2BYTES_TO_FLOAT(canFrame->data[4],canFrame->data[5]) * 0.1;
    *contactorRequest = CAN_2BYTES_TO_UINT16(canFrame->data[6],canFrame->data[7]) & 0xFF;


}



void getChargeCommandSIMUL(Charge_Command *command, float *voltageSetPoint, float *currentSetPoint, __u16 *contactorRequest){

/*
    switch(CAN_2BYTES_TO_UINT16(canFrame->data[0],canFrame->data[1])){
        case 0: *command = DISCONNECTION_REQUEST; break;
        case 1: *command = WAIT_REQUEST; break;
        case 2: *command = OPERATION_REQUEST; break;
        case 3: *command = STOP_REQUEST; break;
        case 4: *command = REARM_REQUEST; break;
        default: break;
    }
    */
    *command = OPERATION_REQUEST;
/*
*voltageSetPoint =351.0;
*currentSetPoint = 245.0;
*contactorRequest = 1;
*/

    *voltageSetPoint =850.0;
    *currentSetPoint = 250.0;
    *contactorRequest = 1;

}






/*0x100*/
unsigned int sendAckEvent(EventWord *eventList){
    struct can_frame canFrame;

    canFrame.can_id = 0x100 + own_node;
    canFrame.data[0] = ((int)(eventList->eventLocation.hardware)) << 4;
    canFrame.data[0] |= ((int)(eventList->eventLocation.subsystem));
    canFrame.data[1] = ((int)(eventList->eventLocation.subsubsystem)) << 4;
    canFrame.data[1] |= ((int)(eventList->eventLocation.severity));

    canFrame.data[2] = ((int)(eventList->eventLocation.eventCode)) ;

    canFrame.data[3] = ((int)(eventList->epochTime >> 24)) & 0xFF;
    canFrame.data[4] = ((int)(eventList->epochTime >> 16)) & 0xFF;
    canFrame.data[5] = ((int)(eventList->epochTime >> 8)) & 0xFF;
    canFrame.data[6] = ((int)(eventList->epochTime)) & 0xFF;

    canFrame.data[7] = ((int)(eventList->uniqueIdentifier)) & 0xFF;
    canFrame.can_dlc = 8;
    return canTransmitStdMsg(&canFrame);
}

void getAckEvent(const struct can_frame *canFrame, EventWord *eventList){
    if ((canFrame->can_id & 0x100) != 0x100 ||  canFrame->can_dlc != 8) return;

    eventList->eventLocation.hardware = (canFrame->data[0] & 0xF0)>>4;
    eventList->eventLocation.subsystem = (canFrame->data[0] & 0x0F);
    eventList->eventLocation.subsubsystem = (canFrame->data[1] & 0xF0)>>4;

    switch(canFrame->data[1] & 0x0F)
    {
        case 0: eventList->eventLocation.severity = SEVERITY_TRACE; break;
        case 1: eventList->eventLocation.severity = SEVERITY_DEBUG; break;
        case 2: eventList->eventLocation.severity = SEVERITY_INFO; break;
        case 3: eventList->eventLocation.severity = SEVERITY_WARNING; break;
        case 4: eventList->eventLocation.severity = SEVERITY_ERROR; break;
        case 5: eventList->eventLocation.severity = SEVERITY_FATAL; break;
    }

    eventList->eventLocation.eventCode = canFrame->data[2];
    eventList->epochTime = (((__u32)canFrame->data[3])<<24) + (((__u32)canFrame->data[4])<<16) +\
            (((__u32)canFrame->data[5])<<8) + (((__u32)canFrame->data[6]));
    eventList->uniqueIdentifier = canFrame->data[7];
}

unsigned char updateLastErrorIdentifier(const struct can_frame *canFrame){
    static unsigned char lastID = 0;

    if ((canFrame->can_id & 0x100) == 0x100 ||  (canFrame->can_id & 0x140) == 0x140) {
        if (canFrame->data[7] > lastID) lastID = canFrame->data[7];
    }
    else return lastID;

    if (canFrame->data[7] < 0x0F && lastID > 0xF0) lastID = canFrame->data[7]; // Identifier overflow

    return lastID;
}

unsigned char updateLastErrorIdentifierM(const struct can_frame *canFrame){
    static unsigned char lastIDM = 0;

    if ((canFrame->can_id & 0x100) == 0x100 ||  (canFrame->can_id & 0x140) == 0x140) {
        if (canFrame->data[7] > lastIDM) lastIDM = canFrame->data[7];
    }
    else return lastIDM;

    if (canFrame->data[7] < 0x0F && lastIDM > 0xF0) lastIDM = canFrame->data[7]; // Identifier overflow

    return lastIDM;
}

unsigned int publishErrorModbus(__u8 error1, __u8 error2, int deltaerror)
{
    struct can_frame canFrame;

    canFrame.can_id = 0x400 + own_node;
    canFrame.data[0] = error1;
    canFrame.data[1] = error2;
    canFrame.data[2] = deltaerror;
    canFrame.data[3] = 0;
    canFrame.data[4] = 0;
    canFrame.data[5] = 0;
    canFrame.data[6] = 0;
    canFrame.data[7] = 0;
    canFrame.can_dlc = 8;
    envia_siempre = 1;
    return canTransmitStdMsg(&canFrame);

}

unsigned int publishErrorComms(__u8 error1, __u8 error2, __u8 error3,int alarm,__u8 error4, __u8 error5)
{
    struct can_frame canFrame;

    canFrame.can_id = 0x400 + own_node;
    canFrame.data[0] = error1;
    canFrame.data[1] = error2;
    canFrame.data[2] = error3;
    canFrame.data[3] = (unsigned char) alarm;
    canFrame.data[4] = error4;
    canFrame.data[5] = error5;
    canFrame.data[6] = 0;
    canFrame.data[7] = 0;
    canFrame.can_dlc = 8;

    return canTransmitStdMsg(&canFrame);

}
/*0x180+Node messages.*/
// unsigned int publishConfiguration(const __u8 errorComm1, const __u8 errorComm2,const __u16 targetNode, const __u8 masterNode, const Output_Config configuration, const float powerLimit){
unsigned int publishConfiguration(const __u16 targetNode, const __u8 masterNode, const Output_Config configuration, const float powerLimit, const int mang){
       struct can_frame canFrame;

    canFrame.can_id = 0x180 + own_node;
    canFrame.data[0] = targetNode >> 8;
    canFrame.data[1] = targetNode & 0xFF;
    canFrame.data[2] = masterNode;
    canFrame.data[3] = (__u8)mang;
    canFrame.data[4] = 0;
    canFrame.data[5] = (int)configuration & 0xFF;
    canFrame.data[6] = ((int)(powerLimit*0.01)) >> 8;
    canFrame.data[7] = ((int)(powerLimit*0.01)) & 0xFF;
    canFrame.can_dlc = 8;

    if (own_node == supervisor_node && targetNode == supervisor_node + 1){
        mensajeDeSuper.targetNode = targetNode;
        mensajeDeSuper.masterNode = masterNode;
        mensajeDeSuper.configuration = configuration;
        mensajeDeSuper.powerLimit = powerLimit;
        mensajeDeSuper.manguera = mang;
        mensajeDeSuper.mensaje_rec[PUBLISH_CONFIGURATION] = 1;
    }


    return canTransmitStdMsg(&canFrame);
}


/*0x180+Node messages.*/
void getConfiguration(const struct can_frame *canFrame, /*const __u16 targetNode,*/ __u8 *linkedNode, Output_Config *configuration, float *powerLimit, int *mang){

    if ((canFrame->can_id & 0x180) != 0x180     || canFrame->can_dlc != 8) return;
    if ((CAN_2BYTES_TO_UINT16(canFrame->data[0],canFrame->data[1]) & 0x007F) != own_node  ) return;

    *linkedNode = canFrame->data[2];
    switch(CAN_2BYTES_TO_UINT16(canFrame->data[4],canFrame->data[5])){
        case 0: *configuration = NOT_CONNECTED; break;
        case 1: *configuration = SOCKET; break;
        case 2: *configuration = SUPPORT; break;
        case 3: *configuration = STANDBY; break;
        case 4: *configuration = NOSTANDBY; break;

    }
    *powerLimit = CAN_2BYTES_TO_FLOAT(canFrame->data[6],canFrame->data[7]) * 100.0;
    *mang = (int)canFrame->data[3];
}

// Actions with errors:
void clearEventBuffer(EventWord e[])
{
    int i;
    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++){
        e[i].eventLocation.hardware = 0;
        e[i].eventLocation.subsystem = 0;
        e[i].eventLocation.subsubsystem = 0;
        e[i].eventLocation.severity = SEVERITY_TRACE;
        e[i].eventLocation.eventCode = 0;
        e[i].uniqueIdentifier = 0;
        e[i].epochTime = 0;
        e[i].lastAccessed = 0;
    }
}

// This function adds an event to the cue, but its previous presence should be checked before using "registerEventLocation".
void appendEvent(EventWord evenToAppend, EventWord listOfEvents[])
{
    int i, writingCandidate = 0;
    __u32 oldestEpochFound = 0xFFFFFFFF; //Less is older
    // Search a void event buffer, or the oldest entry instead.
    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++){
        if(eventIsVoid(&listOfEvents[i])){ //Means void
            writingCandidate = i;
            break; //stop searching
        }
        else
        {
            if (oldestEpochFound > listOfEvents[i].lastAccessed){ //Keep older
                oldestEpochFound = listOfEvents[i].lastAccessed;
                writingCandidate = i;
            }
        }
    }
    listOfEvents[writingCandidate].eventLocation.hardware        = evenToAppend.eventLocation.hardware;
    listOfEvents[writingCandidate].eventLocation.subsystem       = evenToAppend.eventLocation.subsystem;
    listOfEvents[writingCandidate].eventLocation.subsubsystem    = evenToAppend.eventLocation.subsubsystem;
    listOfEvents[writingCandidate].eventLocation.severity        = evenToAppend.eventLocation.severity;
    listOfEvents[writingCandidate].eventLocation.eventCode       = evenToAppend.eventLocation.eventCode;
    listOfEvents[writingCandidate].uniqueIdentifier              = evenToAppend.uniqueIdentifier;
    listOfEvents[writingCandidate].epochTime                     = evenToAppend.epochTime;
    listOfEvents[writingCandidate].lastAccessed                  = systemEpoch; // The current epoch is used as acknowledge
}


void removeEvent(EventLocation *event, EventWord listOfEvents[]){
    int i;
    // Search a void event buffer, or the oldest entry instead.
    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++){
        if (
            listOfEvents[i].eventLocation.hardware        == event->hardware && \
            listOfEvents[i].eventLocation.subsystem       == event->subsystem && \
            listOfEvents[i].eventLocation.subsubsystem    == event->subsubsystem && \
            listOfEvents[i].eventLocation.severity        == event->severity /*&& \
            listOfEvents[i].eventLocation.eventCode       == event->eventCode*/
            )
        {
            listOfEvents[i].eventLocation.hardware = 0;
            listOfEvents[i].eventLocation.subsystem = 0;
            listOfEvents[i].eventLocation.subsubsystem = 0;
            listOfEvents[i].eventLocation.severity = SEVERITY_TRACE;
            listOfEvents[i].eventLocation.eventCode = 0;
            listOfEvents[i].uniqueIdentifier = 0;
            listOfEvents[i].epochTime = 0;
            listOfEvents[i].lastAccessed = 0;
            return;
        }
    }
}

void showEvent(EventWord listOfEvents[])
    {
    int i;
		// Search a void event buffer, or the oldest entry instead.
		for (i = 0; i < 5; i++)
		{

		    if(eventIsVoid(&listOfEvents[i]) == FALSE) //Means void
		    {
#if	DEBUG
				sprintf(bufferstr, "Event: %i hard:%i  sub1:%i sub2:%i sever:%i code:%i e:%i l:%i\n", i, (int) listOfEvents[i].eventLocation.hardware,(int)listOfEvents[i].eventLocation.subsystem,(int)listOfEvents[i].eventLocation.subsubsystem,(int)listOfEvents[i].eventLocation.severity,(unsigned int) listOfEvents[i].eventLocation.eventCode,(int) listOfEvents[i].epochTime,(int) listOfEvents[i].lastAccessed);

				env_debug(bufferstr);
				env_debug("\n");
#endif
		    }
		}

    }

void sortEvents(EventWord listOfEvents[])
{
  int i;
  // Search a void event buffer
  for (i = 0; i < (MAXIMUM_ERRORS_LISTED-1); i++)
  {
    if(eventIsVoid(&listOfEvents[i]) == 1) //Means void
    {
      if(eventIsVoid(&listOfEvents[i+1]) == 0) //Means that the next event is not void
      {
        //Copy the event:
#if	DEBUG
		env_debug("ordenamos evento\n");
#endif
        listOfEvents[i].eventLocation.hardware        = listOfEvents[i+1].eventLocation.hardware;
        listOfEvents[i].eventLocation.subsystem       = listOfEvents[i+1].eventLocation.subsystem;
        listOfEvents[i].eventLocation.subsubsystem    = listOfEvents[i+1].eventLocation.subsubsystem;
        listOfEvents[i].eventLocation.severity        = listOfEvents[i+1].eventLocation.severity;
        listOfEvents[i].eventLocation.eventCode       = listOfEvents[i+1].eventLocation.eventCode;
        listOfEvents[i].uniqueIdentifier              = listOfEvents[i+1].uniqueIdentifier;
        listOfEvents[i].epochTime                     = listOfEvents[i+1].epochTime;
        listOfEvents[i].lastAccessed                  = listOfEvents[i+1].lastAccessed;
        //Remove the moved event:
        listOfEvents[i+1].eventLocation.hardware = 0;
        listOfEvents[i+1].eventLocation.subsystem = 0;
        listOfEvents[i+1].eventLocation.subsubsystem = 0;
        listOfEvents[i+1].eventLocation.severity = SEVERITY_TRACE;
        listOfEvents[i+1].eventLocation.eventCode = 0;
        listOfEvents[i+1].uniqueIdentifier = 0;
        listOfEvents[i+1].epochTime = 0;
        listOfEvents[i+1].lastAccessed = 0;
      }
    }
  }
}

// The update policy is on regards the time spend since last activation and severity
void updateEvents(EventWord listOfEvents[]){
    int i;
    unsigned long elapsedTime;
    unsigned char oneNodeIsCritical = 0;
    unsigned char masterIsCritical = 0;

    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++)
    {
        elapsedTime = systemEpoch - listOfEvents[i].lastAccessed;
        if(elapsedTime > ERROR_SECONDS_TO_CLEAR && listOfEvents[i].eventLocation.severity != SEVERITY_FATAL && listOfEvents[i].epochTime > 0) //Number of seconds
        {
#if	DEBUG
			sprintf(bufferstr, "Registro borrado en: %i", (int)systemEpoch);
			env_debug(bufferstr);
			env_debug("\n");

			sprintf(bufferstr, "Event: %i hard:%i  sub1:%i sub2:%i sever:%i code:%i e:%i l:%i\n", i, (int) listOfEvents[i].eventLocation.hardware,(int)listOfEvents[i].eventLocation.subsystem,(int)listOfEvents[i].eventLocation.subsubsystem,(int)listOfEvents[i].eventLocation.severity,(unsigned int) listOfEvents[i].eventLocation.eventCode,(int) listOfEvents[i].epochTime,(int) listOfEvents[i].lastAccessed);

			env_debug(bufferstr);
			env_debug("\n");
#endif

            listOfEvents[i].eventLocation.hardware = 0;
            listOfEvents[i].eventLocation.subsystem = 0;
            listOfEvents[i].eventLocation.subsubsystem = 0;
            listOfEvents[i].eventLocation.severity = SEVERITY_TRACE;
            listOfEvents[i].eventLocation.eventCode = 0;
            listOfEvents[i].uniqueIdentifier = 0;
            listOfEvents[i].epochTime = 0;
            listOfEvents[i].lastAccessed = 0;
        }

        if(listOfEvents[i].eventLocation.severity == SEVERITY_FATAL)
        {
            oneNodeIsCritical = 1;
        }

        if(listOfEvents[i].eventLocation.severity == SEVERITY_ERROR
          && listOfEvents[i].eventLocation.hardware == supervisor_node)
        {
            masterIsCritical = 1;
        }
    }
    networkIsCriticalFlag = oneNodeIsCritical;
    masterHasErrorsFlag = masterIsCritical;
}

void registerEventLocation(EventLocation *eventL, EventWord listOfEvents[]){
    EventWord newEvent;
    unsigned int elementFound = 0;
    unsigned int i, foundInIndex = 0;
    newEvent.eventLocation.hardware = eventL->hardware;
    newEvent.eventLocation.subsystem = eventL->subsystem;
    newEvent.eventLocation.subsubsystem = eventL->subsubsystem;
    newEvent.eventLocation.severity = eventL->severity;
    newEvent.eventLocation.eventCode = eventL->eventCode;

    // Check if the event already exists
    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++){
       if (
           listOfEvents[i].eventLocation.hardware        == newEvent.eventLocation.hardware && \
           listOfEvents[i].eventLocation.subsystem       == newEvent.eventLocation.subsystem && \
           listOfEvents[i].eventLocation.subsubsystem    == newEvent.eventLocation.subsubsystem && \
           listOfEvents[i].eventLocation.severity        == newEvent.eventLocation.severity && \
           listOfEvents[i].eventLocation.eventCode       == newEvent.eventLocation.eventCode
           )
       {
           elementFound = 1;
           foundInIndex = i;
           break;
       }
    }

    // Update accesses
    newEvent.epochTime = systemEpoch;
    newEvent.lastAccessed = systemEpoch;

    if (elementFound) listOfEvents[foundInIndex].lastAccessed = newEvent.lastAccessed;
    else appendEvent(newEvent, listOfEvents);
}

void registerEventWord(EventWord *eventW, EventWord listOfEvents[]){
    EventWord newEvent;
    unsigned int elementFound = 0;
    unsigned int i, foundInIndex = 0;
    newEvent.eventLocation.hardware = eventW->eventLocation.hardware;
    newEvent.eventLocation.subsystem = eventW->eventLocation.subsystem;
    newEvent.eventLocation.subsubsystem = eventW->eventLocation.subsubsystem;
    newEvent.eventLocation.severity = eventW->eventLocation.severity;
    newEvent.eventLocation.eventCode = eventW->eventLocation.eventCode;

    // Check if the event already exists
    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++){
       if (
           listOfEvents[i].eventLocation.hardware        == newEvent.eventLocation.hardware && \
           listOfEvents[i].eventLocation.subsystem       == newEvent.eventLocation.subsystem && \
           listOfEvents[i].eventLocation.subsubsystem    == newEvent.eventLocation.subsubsystem && \
           listOfEvents[i].eventLocation.severity        == newEvent.eventLocation.severity && \
           listOfEvents[i].eventLocation.eventCode       == newEvent.eventLocation.eventCode
           )
       {
           elementFound = 1;
           foundInIndex = i;
           break;
       }
    }

    // Update accesses
    newEvent.epochTime = systemEpoch;
    newEvent.lastAccessed = systemEpoch;



    //TXEMA ->     >SEVERITY_WARNING?
    if (eventW->eventLocation.severity > SEVERITY_WARNING){
      if (elementFound) listOfEvents[foundInIndex].lastAccessed = newEvent.lastAccessed;
      else appendEvent(newEvent, listOfEvents);

    }
}

unsigned int eventIsVoid(EventWord *eventToBeChecked){
    if (eventToBeChecked->epochTime == 0) return 1;
    else return 0;
}

// Implements a round robin list. Msg 0x900 is the lowest priority msg.
unsigned int publishDeltaData(void)
{
  /* Call the function that takes the data from the local dictionary */
  struct can_frame canFrame;
  queryDeltaData(&canFrame);
  return canTransmitStdMsg(&canFrame);
}

void getDataDeltas(const struct can_frame *canFrame, unsigned int *id1, unsigned int *data1, unsigned int *id2, unsigned int *data2)
{
  *id1 = CAN_2BYTES_TO_UINT16(canFrame->data[0],canFrame->data[1]);
  *data1 = CAN_2BYTES_TO_UINT16(canFrame->data[2],canFrame->data[3]);
  *id2 = CAN_2BYTES_TO_UINT16(canFrame->data[4],canFrame->data[5]);
  *data2 = CAN_2BYTES_TO_UINT16(canFrame->data[6],canFrame->data[7]);
}
